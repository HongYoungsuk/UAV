#pragma once

#include <memory>

#include <irUtils/Conversion.h>
#include <irMath/Common.h>
#include <irMath/LieGroup.h>
#include <irDyn/SerialOpenChain.h>
#include <UAVTrajectory/PTPOptimization.h>

#include <osg/io_utils>
#include <osg/ShapeDrawable>
#include <osg/MatrixTransform>
#include <osgViewer/Viewer>
#include <osgUtil/Optimizer>
#include <osgGA/TerrainManipulator>
#include <osgViewer/ViewerEventHandlers>
#include <osgDB/ReadFile>
#include <osg/LineWidth>
#include <osg/Material>
#include <osg/Shape>

#include "OSG_NodeVisitor.h"
#include "OSG_Primitives.h"
#include "OSG_ExtFile.h"

namespace irLib
{
	namespace irRenderer
	{
		class OSG_simpleRender
		{
			class SimpleGUIHandler : public osgGA::GUIEventHandler
			{
			public:
				SimpleGUIHandler() : _mouseRightButton(false) {}
				~SimpleGUIHandler() {}

				bool handle(const osgGA::GUIEventAdapter& ea, osgGA::GUIActionAdapter& aa);

			private:
				bool _mouseRightButton;
			};

		public:
			typedef std::pair< osg::ref_ptr< osg::MatrixTransform >, const irDyn::LinkState* > NodeStatePair;
			OSG_simpleRender(const irDyn::SerialOpenChainPtr assem, const irDyn::StatePtr state, int width, int height);

			osg::ref_ptr< osg::Group >&	getRoot() { return _rootNode; }
			osgViewer::Viewer&	getViewer() { return _viewer; }
			void updateFrame();


			void addGeometry(const Primitives& geom) { _geometryNode->addDrawable(geom.getGeometry()); }
			void removeGeometry(const Primitives& geom) { _geometryNode->removeDrawable(geom.getGeometry()); }

			void add(const ExtFile& file) { _rootNode->addChild(file.getRoot()); }
			void remove(const ExtFile& file) { _rootNode->removeChild(file.getRoot()); }

		public:
			static const unsigned int numTiles = 25;
			static osg::ref_ptr< osg::Node > createGround(const float& size = (1.0f));

			osg::ref_ptr< osg::Group >					_rootNode;
			osgViewer::Viewer							_viewer;
			osg::ref_ptr< osgGA::TerrainManipulator >	_cameraManipulator;
			osgUtil::Optimizer							_optimzer;
			osg::ref_ptr<osg::Geode>					_geometryNode;
			std::vector< NodeStatePair >				_NodeStateList;
		};

		class UAV_SimpleRender
		{
		public:
			UAV_SimpleRender(UAVTG::UAVTrajectory::PTPOptimization* optimizer, int width = 600, int height = 600);
			osgViewer::Viewer&	getViewer() { return _viewer; }

			void addGeometry(const Primitives& geom) { _geometryNode->addDrawable(geom.getGeometry()); }
			void removeGeometry(const Primitives& geom) { _geometryNode->removeDrawable(geom.getGeometry()); }
			void addLine(const std::vector<osg::Vec3>& linePoints, const osg::Vec4& color = osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f), const float width = (3.0f));
			void addPoint(const osg::Vec3& center, const float size, const osg::Vec4& color = osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
			void addSphere(const osg::Vec3& center, const float radius, const osg::Vec4& color = osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
		private:
			void createAxis();
			void createInitialFinalPoints(UAVTG::UAVTrajectory::PTPOptimization* optimizer);
			void createLines(UAVTG::UAVTrajectory::PTPOptimization* optimizer);
			void createSphereObstacles(UAVTG::UAVTrajectory::PTPOptimization* optimizer);

			osg::ref_ptr< osg::Group >					_rootNode;
			osg::ref_ptr< osg::Geode >					_geometryNode;
			osg::ref_ptr< osgGA::TerrainManipulator >	_cameraManipulator;
			osgViewer::Viewer							_viewer;		
		};
	}
}