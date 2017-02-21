#include "OSG_SimpleRender.h"
#include <osgUtil/IncrementalCompileOperation>

#include <cmath>

using namespace irLib::irDyn;
using namespace irLib::irMath;

namespace irLib
{
	namespace irRenderer
	{
		osg::Matrix convertMatrix(const SE3 T)
		{
			Matrix3 R = T.getRotation().matrix();
			Vector3 p = T.getPosition();

			return osg::Matrix((float)R(0, 0), (float)R(1, 0), (float)R(2, 0), 0.0f,
				(float)R(0, 1), (float)R(1, 1), (float)R(2, 1), 0.0f,
				(float)R(0, 2), (float)R(1, 2), (float)R(2, 2), 0.0f,
				(float)p(0), (float)p(1), (float)p(2), 1.0f);
		}

		osg::ref_ptr< osg::Node > convertGeo2Node(const GeometryInfoPtr geoPtr)
		{
			osg::ref_ptr< osg::MatrixTransform > linkPositionTransform;
			osg::ref_ptr< osg::MatrixTransform > scaleTransform;

			switch (geoPtr->getType())
			{
			case GeometryInfo::GEOMETRY_TYPE::_BOX:
			{
				linkPositionTransform = new osg::MatrixTransform;
				linkPositionTransform->setMatrix(convertMatrix(geoPtr->getTransform()));
				Vector3 dim = (std::static_pointer_cast<Box>(geoPtr))->getDimension();
				linkPositionTransform->addChild(new osg::ShapeDrawable(new osg::Box(osg::Vec3(), (float)dim(1), (float)dim(0), (float)dim(2))));
				return linkPositionTransform;
			}

			case GeometryInfo::GEOMETRY_TYPE::_SPHERE:
			{
				linkPositionTransform = new osg::MatrixTransform;
				linkPositionTransform->setMatrix(convertMatrix(geoPtr->getTransform()));
				linkPositionTransform->addChild(new osg::ShapeDrawable(new osg::Sphere(osg::Vec3(), (float)(std::static_pointer_cast<Sphere>(geoPtr))->getRadius())));
				return linkPositionTransform;
			}

			case GeometryInfo::GEOMETRY_TYPE::_CAPSULE:
			{
				linkPositionTransform = new osg::MatrixTransform;
				linkPositionTransform->setMatrix(convertMatrix(geoPtr->getTransform()));
				Vector2 dim = (std::static_pointer_cast<Capsule>(geoPtr))->getDimension();
				linkPositionTransform->addChild(new osg::ShapeDrawable(new osg::Capsule(osg::Vec3(), (float)dim(0), (float)dim(1))));
				return linkPositionTransform;
			}

			case GeometryInfo::GEOMETRY_TYPE::_CYLINDER:
			{
				linkPositionTransform = new osg::MatrixTransform;
				linkPositionTransform->setMatrix(convertMatrix(geoPtr->getTransform()));
				Vector2 dim = (std::static_pointer_cast<Cylinder>(geoPtr))->getDimension();
				linkPositionTransform->addChild(new osg::ShapeDrawable(new osg::Cylinder(osg::Vec3(), (float)dim(0), (float)dim(1))));
				return linkPositionTransform;
			}

			case GeometryInfo::GEOMETRY_TYPE::_MESH:
			{
				osg::ref_ptr<osgDB::ReaderWriter::Options> _options = new osgDB::ReaderWriter::Options();
				_options->setObjectCacheHint(osgDB::ReaderWriter::Options::CACHE_ALL);
				auto meshPtr = std::static_pointer_cast<Mesh>(geoPtr);

				linkPositionTransform = new osg::MatrixTransform;
				linkPositionTransform->setMatrix(convertMatrix(geoPtr->getTransform()));

				scaleTransform = new osg::MatrixTransform;
				scaleTransform->setMatrix(osg::Matrixd::scale(meshPtr->getDimension(), meshPtr->getDimension(), meshPtr->getDimension()));
				scaleTransform->getOrCreateStateSet()->setMode(GL_RESCALE_NORMAL, osg::StateAttribute::ON);

				osg::ref_ptr<osg::Node> STL_node = osgDB::readNodeFile(meshPtr->getUrl(), _options);

				STL_node->setCullingActive(true);
				STL_node->setDataVariance(osg::Object::DataVariance::STATIC);
				auto meshColor = meshPtr->getColor();

				OSG_NodeVisitor _nodeVisitor;
				_nodeVisitor.setColor(meshColor[0], meshColor[1], meshColor[2], meshColor[3]);
				STL_node->accept(_nodeVisitor);

				osg::ref_ptr<osg::Material> _material = new osg::Material;
				_material->setColorMode(osg::Material::ColorMode::AMBIENT_AND_DIFFUSE);
				_material->setDiffuse(osg::Material::FRONT, osg::Vec4(0.6, 0.6, 0.6, 0.6));
				_material->setAmbient(osg::Material::FRONT, osg::Vec4(0.1, 0.1, 0.1, 0.1));
				_material->setSpecular(osg::Material::FRONT, osg::Vec4(1.0, 1.0, 1.0, 1.0));
				_material->setShininess(osg::Material::FRONT, 20);
				STL_node->getOrCreateStateSet()->setAttribute(_material);

				scaleTransform->addChild(STL_node);
				linkPositionTransform->addChild(scaleTransform);
				return linkPositionTransform;
			}

			default:
			{
				linkPositionTransform = new osg::MatrixTransform;
				// TODO
				return linkPositionTransform;
			}
			}
		}

		bool OSG_simpleRender::SimpleGUIHandler::handle(const osgGA::GUIEventAdapter &ea, osgGA::GUIActionAdapter &aa)
		{
			switch (ea.getEventType())
			{
			case(osgGA::GUIEventAdapter::DRAG) :
			{
				if (_mouseRightButton)
				{
					return true;
				}
				return false;
			}
			case(osgGA::GUIEventAdapter::PUSH) :
			{
				if (ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON)
				{
					_mouseRightButton = true;
					return true;
				}
				return false;
			}
			case(osgGA::GUIEventAdapter::RELEASE) :
			{
				if (_mouseRightButton)
				{
					_mouseRightButton = false;
				}
				return true;
			}
			default:
				return false;
			}
		}

		OSG_simpleRender::OSG_simpleRender(const SerialOpenChainPtr socRobot, const StatePtr state, int width, int height)
		{
			_rootNode = new osg::Group;
			_cameraManipulator = new osgGA::TerrainManipulator();
			_geometryNode = new osg::Geode();

			_rootNode->addChild(createGround());
			_rootNode->addChild(_geometryNode);

			unsigned int dof = socRobot->getNumOfJoint();

			for (unsigned int i = 0; i < dof + 1; i++)
			{
				osg::ref_ptr< osg::MatrixTransform > transformNode = new osg::MatrixTransform;
				transformNode->setMatrix(convertMatrix(state->getLinkSE3(i)));
				const std::vector<GeometryInfoPtr>& shapes = socRobot->getLinkPtr(i)->getDrawingGeometryInfo();

				for (unsigned int j = 0; j < shapes.size(); j++)
					transformNode->addChild(convertGeo2Node(shapes[j]));

				_NodeStateList.push_back(NodeStatePair(transformNode, &state->getLinkState(i)));
				_rootNode->addChild(transformNode);
			}

			//_optimzer.optimize(_rootNode,osgUtil::Optimizer::DEFAULT_OPTIMIZATIONS);
			//osg::ref_ptr<osgUtil::IncrementalCompileOperation> ico = new osgUtil::IncrementalCompileOperation;
			//ico->add(_rootNode);
			//ico->release();
			//_viewer.setIncrementalCompileOperation(ico);

			_viewer.setUpViewInWindow(40, 40, width, height);
			_viewer.setSceneData(_rootNode);
			_viewer.getCamera()->setClearColor(osg::Vec4(90 / 255.0, 102 / 255.0, 117 / 255.0, 1.0f));
			_viewer.setCameraManipulator(_cameraManipulator.get(), true);
			//_cameraManipulator->setDistance(5.0f);
			//_cameraManipulator->setRotation(osg::Quat(1, osg::Vec3d(1, 0, 0)));
			_cameraManipulator->setDistance(7.0f);
			_cameraManipulator->setRotation(osg::Quat(1, osg::Vec3d(1, 0.5, 0.5)));

			_viewer.addEventHandler(new osgViewer::WindowSizeHandler);
			_viewer.addEventHandler(new SimpleGUIHandler());
		}

		osg::ref_ptr< osg::Node > OSG_simpleRender::createGround(const float& size)
		{
			osg::ref_ptr< osg::Geode > geode = new osg::Geode;
			osg::ref_ptr< osg::Geometry > geom = new osg::Geometry;

			float width = 2 * size;
			float height = 2 * size;

			osg::Vec3 v0(-width*0.5f, -height*0.5f, 0.0f);
			osg::Vec3 dx(width / ((float)(numTiles * 2)), 0.0f, 0.0f);
			osg::Vec3 dy(0.0f, width / ((float)(numTiles * 2)), 0.0f);

			osg::Vec3Array* coords = new osg::Vec3Array;
			for (unsigned int iy = 0; iy <= numTiles * 2; iy++)
			{
				for (unsigned int ix = 0; ix <= numTiles * 2; ix++)
				{
					coords->push_back(v0 + dx*(float)ix + dy*(float)iy);
				}
			}
			geom->setVertexArray(coords);

			osg::ref_ptr< osg::Vec4Array > colors = new osg::Vec4Array;

			int numIndicesPerRow = numTiles * 2 + 1;
			for (unsigned int iy = 0; iy < numTiles; iy++)
			{
				for (unsigned int ix = 0; ix < numTiles; ix++)
				{
					osg::Vec3 color;
					color = ((iy + ix) % 2 == 0) ? osg::Vec3(1.0f, 1.0f, 1.0f) : osg::Vec3(0.2f, 0.2f, 0.2f);

					for (unsigned int sy = iy * 2; sy < (iy + 1) * 2; sy++)
					{
						for (unsigned int sx = ix * 2; sx < (ix + 1) * 2; sx++)
						{
							osg::ref_ptr< osg::DrawElementsUShort > primitives = new osg::DrawElementsUShort(GL_QUADS);
							primitives->push_back(sx + (sy + 1)*numIndicesPerRow);
							primitives->push_back(sx + sy*numIndicesPerRow);
							primitives->push_back((sx + 1) + sy*numIndicesPerRow);
							primitives->push_back((sx + 1) + (sy + 1)*numIndicesPerRow);
							geom->addPrimitiveSet(primitives.get());

							float length = std::sqrtf((float)((sy - numTiles)*(sy - numTiles) + (sx - numTiles)*(sx - numTiles)));
							float kapa = 5.0f / (numTiles*std::sqrtf(2) - numTiles);
							colors->push_back(osg::Vec4(color, 0.7f / (1 + std::expf(-kapa*(numTiles - length)))));
						}
					}
				}
			}

			geom->setColorArray(colors, osg::Array::BIND_PER_PRIMITIVE_SET);

			osg::ref_ptr< osg::Vec3Array > normals = new osg::Vec3Array;
			normals->push_back(osg::Vec3(0.0f, 0.0f, 1.0f));
			geom->setNormalArray(normals, osg::Array::BIND_OVERALL);

			osg::StateSet* stateset = new osg::StateSet;
			stateset->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
			stateset->setMode(GL_BLEND, osg::StateAttribute::ON);
			stateset->setRenderingHint(osg::StateSet::TRANSPARENT_BIN);
			geode->setStateSet(stateset);

			geode->addDrawable(geom);

			return geode;
		}
		void OSG_simpleRender::updateFrame()
		{
			for (unsigned int i = 0; i < _NodeStateList.size(); i++)
			{
				_NodeStateList[i].first->setMatrix(convertMatrix(_NodeStateList[i].second->getLinkSE3()));
			}
			_viewer.frame();
		}

		/*
			UAV_SimpleRenderer class
			made by Youngsuk Hong(crazyhys@gmail.com)
		*/
		UAV_SimpleRender::UAV_SimpleRender(UAVTG::UAVTrajectory::PTPOptimization* optimizer, int width, int height)
		{
			_rootNode = new osg::Group;
			_cameraManipulator = new osgGA::TerrainManipulator();
			_geometryNode = new osg::Geode();;

			_rootNode->addChild(OSG_simpleRender::createGround(5.0f));
			_rootNode->addChild(_geometryNode);

			//createAxis();
			createInitialFinalPoints(optimizer);
			createLines(optimizer);
			createSphereObstacles(optimizer);

			_viewer.setUpViewInWindow(40, 40, width, height);
			_viewer.setSceneData(_rootNode);
			_viewer.getCamera()->setClearColor(osg::Vec4(90 / 255.0, 102 / 255.0, 117 / 255.0, 1.0f));
			//_viewer.getCamera()->setClearColor(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
			_viewer.setCameraManipulator(_cameraManipulator.get(), true);
			//_cameraManipulator->setDistance(5.0f);
			//_cameraManipulator->setRotation(osg::Quat(1, osg::Vec3d(1, 0, 0)));
			_cameraManipulator->setDistance(25.0f);
			_cameraManipulator->setRotation(osg::Quat(1, osg::Vec3d(1, 0.5, 0.5)));

			_viewer.addEventHandler(new osgViewer::WindowSizeHandler);			
		}

		void UAV_SimpleRender::addLine(const std::vector<osg::Vec3>& linePoints, const osg::Vec4& color, const float width)
		{
			std::shared_ptr<Line> line = std::shared_ptr<Line>(new Line);
			for (unsigned int i = 0; i < linePoints.size(); i++)
				line->push_back(linePoints[i]);
			line->setWidth(width);
			line->setColor(color[0], color[1], color[2], color[3]);
			_geometryNode->addDrawable(line->getGeometry());
		}

		void UAV_SimpleRender::addPoint(const osg::Vec3 & center, const float size, const osg::Vec4 & color)
		{
			std::shared_ptr<Points> point = std::shared_ptr<Points>(new Points);
			point->push_back(center);
			point->setSize(size);
			point->setColor(color[0], color[1], color[2], color[3]);
			_geometryNode->addDrawable(point->getGeometry());
		}

		void UAV_SimpleRender::addSphere(const osg::Vec3 & center, const float radius, const osg::Vec4 & color)
		{
			osg::Sphere* unitSphere = new osg::Sphere(center, radius);
			osg::ShapeDrawable* unitSphereDrawable = new osg::ShapeDrawable(unitSphere);
			unitSphereDrawable->setColor(color);
			_geometryNode->addDrawable(unitSphereDrawable);
		}
		void UAV_SimpleRender::createAxis()
		{
			std::vector<osg::Vec3> x_axis, y_axis, z_axis;
			x_axis.push_back(osg::Vec3(0.0, 0.0, 0.0));
			x_axis.push_back(osg::Vec3(0.5f, 0.0, 0.0));

			y_axis.push_back(osg::Vec3(0.0, 0.0, 0.0));
			y_axis.push_back(osg::Vec3(0.0, 0.5f, 0.0));

			z_axis.push_back(osg::Vec3(0.0, 0.0, 0.0));
			z_axis.push_back(osg::Vec3(0.0, 0.0, 0.5f));

			addLine(x_axis, osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f), 3);
			addLine(y_axis, osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f), 3);
			addLine(z_axis, osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f), 3);
		}

		void UAV_SimpleRender::createInitialFinalPoints(UAVTG::UAVTrajectory::PTPOptimization* optimizer)
		{
			//addPoint(optimizer->_initialState.col(0), )
			osg::Vec3 initialPoint, finalPoint;
			for (unsigned int i = 0; i < 3; i++)
			{
				initialPoint[i] = optimizer->_initialState.col(0)(i);
				finalPoint[i] = optimizer->_finalState.col(0)(i);
				//std::cout << finalPoint[i] << std::endl;
			}
			
			addPoint(initialPoint, 10, osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
			addPoint(finalPoint, 10, osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
		}

		void UAV_SimpleRender::createLines(UAVTG::UAVTrajectory::PTPOptimization* optimizer)
		{
			std::vector<osg::Vec3> initialLine, finalLine;
			for (unsigned int i = 0; i < optimizer->_initialTrajectory.size(); i++)
			{
				initialLine.push_back(osg::Vec3(optimizer->_initialTrajectory[i](0), optimizer->_initialTrajectory[i](1), optimizer->_initialTrajectory[i](2)));
				finalLine.push_back(osg::Vec3(optimizer->_finalTrajectory[i](0), optimizer->_finalTrajectory[i](1), optimizer->_finalTrajectory[i](2)));
			}
			addLine(initialLine, osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
			addLine(finalLine, osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));
		}

		void UAV_SimpleRender::createSphereObstacles(UAVTG::UAVTrajectory::PTPOptimization* optimizer)
		{
			for (unsigned int i = 0; i < optimizer->_SphereObstacleConFunc.size(); i++)
			{
				osg::Vec3 center;
				float radius;
				center[0] = std::static_pointer_cast<UAVTG::UAVTrajectory::SphereObstacleConstraint>(optimizer->_SphereObstacleConFunc[i])->_center(0);
				center[1] = std::static_pointer_cast<UAVTG::UAVTrajectory::SphereObstacleConstraint>(optimizer->_SphereObstacleConFunc[i])->_center(1);
				center[2] = std::static_pointer_cast<UAVTG::UAVTrajectory::SphereObstacleConstraint>(optimizer->_SphereObstacleConFunc[i])->_center(2);
				radius = std::static_pointer_cast<UAVTG::UAVTrajectory::SphereObstacleConstraint>(optimizer->_SphereObstacleConFunc[i])->_radius;

				addSphere(center, radius);
			}
		}
	}
}