/* OpenSceneGraph example, osgbillboard.
2	*
3	*  Permission is hereby granted, free of charge, to any person obtaining a copy
4	*  of this software and associated documentation files (the "Software"), to deal
5	*  in the Software without restriction, including without limitation the rights
6	*  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
7	*  copies of the Software, and to permit persons to whom the Software is
8	*  furnished to do so, subject to the following conditions:
9	*
10	*  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
11	*  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
12	*  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
13	*  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
14	*  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
15	*  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
16	*  THE SOFTWARE.
17	*/

#include <iostream>
#include <conio.h>

#include <osg/Node>
#include <osg/Geometry>
#include <osg/Notify>
#include <osg/MatrixTransform>
#include <osg/Texture2D>
#include <osg/Billboard>
#include <osg/LineWidth>

#include <osgGA/TrackballManipulator>
#include <osgGA/FlightManipulator>
#include <osgGA/DriveManipulator>

#include <osgDB/Registry>
#include <osgDB/ReadFile>

#include <osgViewer/Viewer>

#include <irRenderer\OSG_SimpleRender.h>

//
// A simple demo demonstrating different texturing modes,
// including using of texture extensions.
//


typedef std::vector< osg::ref_ptr<osg::Image> > ImageList;

/** create quad at specified position. */
osg::Drawable* createSquare(const osg::Vec3& corner,const osg::Vec3& width,const osg::Vec3& height, osg::ref_ptr<osg::Image> image)
{
    // set up the Geometry.
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;

    osg::Vec3Array* coords = new osg::Vec3Array(4);
    (*coords)[0] = corner;
    (*coords)[1] = corner+width;
    (*coords)[2] = corner+width+height;
    (*coords)[3] = corner+height;


    geom->setVertexArray(coords);

    osg::Vec3Array* norms = new osg::Vec3Array(1);
    (*norms)[0] = width^height;
    (*norms)[0].normalize();

    geom->setNormalArray(norms, osg::Array::BIND_OVERALL);

    osg::Vec2Array* tcoords = new osg::Vec2Array(4);
    (*tcoords)[0].set(0.0f,0.0f);
    (*tcoords)[1].set(1.0f,0.0f);
    (*tcoords)[2].set(1.0f,1.0f);
    (*tcoords)[3].set(0.0f,1.0f);
    geom->setTexCoordArray(0,tcoords);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS,0,4));

    if (image)
    {
        osg::StateSet* stateset = new osg::StateSet;
        osg::Texture2D* texture = new osg::Texture2D;
        texture->setImage(image);
        stateset->setTextureAttributeAndModes(0,texture,osg::StateAttribute::ON);
        geom->setStateSet(stateset);
    }

    return geom.release();
}

osg::Drawable* createAxis(const osg::Vec3& corner,const osg::Vec3& xdir,const osg::Vec3& ydir,const osg::Vec3& zdir)
{
    // set up the Geometry.
    osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;

    osg::Vec3Array* coords = new osg::Vec3Array(6);
    (*coords)[0] = corner;
    (*coords)[1] = corner+xdir;
    (*coords)[2] = corner;
    (*coords)[3] = corner+ydir;
    (*coords)[4] = corner;
    (*coords)[5] = corner+zdir;

    geom->setVertexArray(coords);

    osg::Vec4 x_color(0.0f,1.0f,1.0f,1.0f);
    osg::Vec4 y_color(0.0f,1.0f,1.0f,1.0f);
    osg::Vec4 z_color(1.0f,0.0f,0.0f,1.0f);

    osg::Vec4Array* color = new osg::Vec4Array(6);
    (*color)[0] = x_color;
    (*color)[1] = x_color;
    (*color)[2] = y_color;
    (*color)[3] = y_color;
    (*color)[4] = z_color;
    (*color)[5] = z_color;

    geom->setColorArray(color, osg::Array::BIND_PER_VERTEX);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES,0,6));

    osg::StateSet* stateset = new osg::StateSet;
    osg::LineWidth* linewidth = new osg::LineWidth();
    linewidth->setWidth(6.0f);
    stateset->setAttributeAndModes(linewidth,osg::StateAttribute::ON);
    stateset->setMode(GL_LIGHTING,osg::StateAttribute::OFF);
    geom->setStateSet(stateset);

    return geom.release();
}

osg::ref_ptr<osg::Node> createModel()
{

    // create the root node which will hold the model.
    osg::ref_ptr<osg::Group> root = new osg::Group();

    // add the drawable into a single geode to be shared...
    osg::Billboard* center = new osg::Billboard();
    center->setMode(osg::Billboard::POINT_ROT_EYE);
    center->addDrawable(
        createSquare(osg::Vec3(-0.5f,0.0f,-0.5f),osg::Vec3(1.0f,0.0f,0.0f),osg::Vec3(0.0f,0.0f,1.0f),osgDB::readRefImageFile("Images/reflect.rgb")),
        osg::Vec3(0.0f,0.0f,0.0f));

    osg::Billboard* x_arrow = new osg::Billboard();
    x_arrow->setMode(osg::Billboard::AXIAL_ROT);
    x_arrow->setAxis(osg::Vec3(1.0f,0.0f,0.0f));
    x_arrow->setNormal(osg::Vec3(0.0f,-1.0f,0.0f));
    x_arrow->addDrawable(
       createSquare(osg::Vec3(-0.5f,0.0f,-0.5f),osg::Vec3(1.0f,0.0f,0.0f),osg::Vec3(0.0f,0.0f,1.0f),osgDB::readRefImageFile("Cubemap_axis/posx.png")),
       osg::Vec3(5.0f,0.0f,0.0f));

    osg::Billboard* y_arrow = new osg::Billboard();
    y_arrow->setMode(osg::Billboard::AXIAL_ROT);
    y_arrow->setAxis(osg::Vec3(0.0f,1.0f,0.0f));
    y_arrow->setNormal(osg::Vec3(1.0f,0.0f,0.0f));
    y_arrow->addDrawable(
        createSquare(osg::Vec3(0.0f,-0.5f,-0.5f),osg::Vec3(0.0f,1.0f,0.0f),osg::Vec3(0.0f,0.0f,1.0f),osgDB::readRefImageFile("Cubemap_axis/posy.png")),
        osg::Vec3(0.0f,5.0f,0.0f));

    osg::Billboard* z_arrow = new osg::Billboard();
    z_arrow->setMode(osg::Billboard::AXIAL_ROT);
    z_arrow->setAxis(osg::Vec3(0.0f,0.0f,1.0f));
    z_arrow->setNormal(osg::Vec3(0.0f,-1.0f,0.0f));
    z_arrow->addDrawable(
        createSquare(osg::Vec3(-0.5f,0.0f,-0.5f),osg::Vec3(1.0f,0.0f,0.0f),osg::Vec3(0.0f,0.0f,1.0f),osgDB::readRefImageFile("Cubemap_axis/posz.png")),
        osg::Vec3(0.0f,0.0f,5.0f));



    osg::Geode* axis = new osg::Geode();
    axis->addDrawable(createAxis(osg::Vec3(0.0f,0.0f,0.0f),osg::Vec3(5.0f,0.0f,0.0f),osg::Vec3(0.0f,5.0f,0.0f),osg::Vec3(0.0f,0.0f,5.0f)));


    root->addChild(center);
    root->addChild(x_arrow);
    root->addChild(y_arrow);
    root->addChild(z_arrow);
    root->addChild(axis);

    return root;
}

int main(int, char**)
{
	irLib::irRenderer::UAV_SimpleRender renderer(600, 600);

	std::vector<osg::Vec3> line;
	line.push_back(osg::Vec3(0.0, 0.0, 0.0));
	line.push_back(osg::Vec3(0.0, 0.0, 1.0));
	//line.push_back(osg::Vec3(0.1742, 0.1757, 0.5353));
	//line.push_back(osg::Vec3(1.367589491387995, 1.366507928104540, 2.348736238754690));
	//line.push_back(osg::Vec3(2.677537008769107, 2.673642062782784, 4.255739708060048));
	//line.push_back(osg::Vec3(2.999960158754533, 2.999959020807381, 4.999851843957098));

	renderer.addLine(line);
	renderer.addSphere(osg::Vec3(0.0, 0.0, 0.5), 0.5, osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));
	renderer.getViewer().run();
	std::cout << "Program Complete" << std::endl;
	_getch();
	return 0;

 //   // construct the viewer
 //   osgViewer::Viewer viewer;
 //   //viewer.setSceneData(createModel());
	//viewer.setUpViewInWindow(40, 40, 600, 600);

 //   // run the viewers frame loop
 //   return viewer.run();
}

//#include <conio.h>
//#include <osg/Node>
//#include <osg/Group>
//#include <osg/Geode>
//#include <osg/Geometry>
//#include <osg/Texture2D>
//#include <osg/ShapeDrawable>
//#include <osg/TexEnv>
//#include <osgDB/ReadFile> 
//#include <osgViewer/Viewer>
//#include <osg/PositionAttitudeTransform>
//#include <osgGA/TrackballManipulator>
//
//int main()
//{
//	// Declare a group to act as root node of a scene:
//	osg::Group* root = new osg::Group();
//
//	// Declare a box class (derived from shape class) instance
//	// This constructor takes an osg::Vec3 to define the center
//	// and a float to define the height, width and depth.
//	// (an overloaded constructor allows you to specify unique
//	// height, width and height values.)
//	osg::Box* unitCube = new osg::Box(osg::Vec3(0, 0, 0), 1.0f);
//
//	// Declare an instance of the shape drawable class and initialize 
//	// it with the unitCube shape we created above.
//	// This class is derived from 'drawable' so instances of this
//	// class can be added to Geode instances.
//	osg::ShapeDrawable* unitCubeDrawable = new osg::ShapeDrawable(unitCube);
//
//	// Declare a instance of the geode class: 
//	osg::Geode* basicShapesGeode = new osg::Geode();
//
//	// Add the unit cube drawable to the geode:
//	basicShapesGeode->addDrawable(unitCubeDrawable);
//
//	// Add the goede to the scene:
//	root->addChild(basicShapesGeode);
//
//	// Create a sphere centered at the origin, unit radius: 
//	osg::Sphere* unitSphere = new osg::Sphere(osg::Vec3(0, 0, 0), 1.0);
//	osg::ShapeDrawable* unitSphereDrawable = new osg::ShapeDrawable(unitSphere);
//
//	osg::PositionAttitudeTransform* sphereXForm = new osg::PositionAttitudeTransform();
//	sphereXForm->setPosition(osg::Vec3(2.5, 0, 0));
//
//	osg::Geode* unitSphereGeode = new osg::Geode();
//	root->addChild(sphereXForm);
//
//	sphereXForm->addChild(unitSphereGeode);
//	unitSphereGeode->addDrawable(unitSphereDrawable);
//
//	osg::Vec4Array* colors = new osg::Vec4Array;
//	colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f)); //index 0 red
//	unitSphereDrawable->setColor(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f));
//
//	// Create line
//	
//
//	//// Declare a state set for 'BLEND' texture mode
//	//osg::StateSet* blendStateSet = new osg::StateSet();
//
//	//// Declare a TexEnv instance, set the mode to 'BLEND'
//	//osg::TexEnv* blendTexEnv = new osg::TexEnv;
//	//blendTexEnv->setMode(osg::TexEnv::BLEND);
//
//	//// Turn the attribute of texture 0 - the texture we loaded above - 'ON'
//	////blendStateSet->setTextureAttributeAndModes(0, KLN89FaceTexture, osg::StateAttribute::ON);
//
//	//// Set the texture texture environment for texture 0 to the 
//	//// texture envirnoment we declared above:
//	//blendStateSet->setTextureAttribute(0, blendTexEnv);
//
//	//osg::StateSet* decalStateSet = new osg::StateSet();
//
//	//osg::TexEnv* decalTexEnv = new osg::TexEnv();
//	//decalTexEnv->setMode(osg::TexEnv::DECAL);
//
//	////decalStateSet->setTextureAttributeAndModes(0, KLN89FaceTexture, osg::StateAttribute::ON);
//	//decalStateSet->setTextureAttribute(0, decalTexEnv);
//
//	//root->setStateSet(blendStateSet);
//	//unitSphereGeode->setStateSet(decalStateSet);
//
//	osgViewer::Viewer viewer;
//
//	viewer.setSceneData(root);
//	viewer.setUpViewInWindow(40, 40, 600, 600);
//
//	return viewer.run();
//
//	std::cout << "Complete Program" << std::endl;
//	_getch();
//	return 0;
//}

//int main()
//{
//	osgViewer::Viewer viewer;
//	osg::Group* root = new osg::Group();
//	osg::Geode* pyramidGeode = new osg::Geode();
//	osg::Geometry* pyramidGeometry = new osg::Geometry();
//	osg::Geode* crossGeode = new osg::Geode();
//	osg::Geometry* crossGeometry = new osg::Geometry();
//
//	//Associate the pyramid geometry with the pyramid geode 
//	//   Add the pyramid geode to the root node of the scene graph.
//
//	pyramidGeode->addDrawable(pyramidGeometry);
//	root->addChild(pyramidGeode);
//	crossGeode->addDrawable(crossGeometry);
//	root->addChild(crossGeode);
//
//	//Declare an array of vertices. Each vertex will be represented by 
//	//a triple -- an instances of the vec3 class. An instance of 
//	//osg::Vec3Array can be used to store these triples. Since 
//	//osg::Vec3Array is derived from the STL vector class, we can use the
//	//push_back method to add array elements. Push back adds elements to 
//	//the end of the vector, thus the index of first element entered is 
//	//zero, the second entries index is 1, etc.
//	//Using a right-handed coordinate system with 'z' up, array 
//	//elements zero..four below represent the 5 points required to create 
//	//a simple pyramid.
//
//	osg::Vec3Array* pyramidVertices = new osg::Vec3Array;
//	pyramidVertices->push_back(osg::Vec3(0, 0, 0)); // front left 
//	pyramidVertices->push_back(osg::Vec3(10, 0, 0)); // front right 
//	pyramidVertices->push_back(osg::Vec3(10, 10, 0)); // back right 
//	pyramidVertices->push_back(osg::Vec3(0, 10, 0)); // back left 
//	pyramidVertices->push_back(osg::Vec3(5, 5, 10)); // peak
//
//	float clen;
//	clen = 12.0;
//	osg::Vec3Array* crossVertices = new osg::Vec3Array;
//	crossVertices->push_back(osg::Vec3(-clen, 0.0, 0.0));
//	crossVertices->push_back(osg::Vec3(clen, 0.0, 0.0));
//	crossVertices->push_back(osg::Vec3(0.0, 0.0, -clen));
//	crossVertices->push_back(osg::Vec3(0.0, 0.0, clen));
//
//	//Associate this set of vertices with the geometry associated with the 
//	//geode we added to the scene.
//
//	pyramidGeometry->setVertexArray(pyramidVertices);
//	crossGeometry->setVertexArray(crossVertices);
//	//Next, create a primitive set and add it to the pyramid geometry. 
//	//Use the first four points of the pyramid to define the base using an 
//	//instance of the DrawElementsUint class. Again this class is derived 
//	//from the STL vector, so the push_back method will add elements in 
//	//sequential order. To ensure proper backface cullling, vertices 
//	//should be specified in counterclockwise order. The arguments for the 
//	//constructor are the enumerated type for the primitive 
//	//(same as the OpenGL primitive enumerated types), and the index in 
//	//the vertex array to start from.
//
//	osg::DrawElementsUInt* pyramidBase =
//		new osg::DrawElementsUInt(osg::PrimitiveSet::QUADS, 0);
//	pyramidBase->push_back(3);
//	pyramidBase->push_back(2);
//	pyramidBase->push_back(1);
//	pyramidBase->push_back(0);
//	pyramidGeometry->addPrimitiveSet(pyramidBase);
//
//	osg::DrawElementsUInt* cross =
//		new osg::DrawElementsUInt(osg::PrimitiveSet::LINES, 0);
//	cross->push_back(3);
//	cross->push_back(2);
//	cross->push_back(1);
//	cross->push_back(0);
//	crossGeometry->addPrimitiveSet(cross);
//
//	//Repeat the same for each of the four sides. Again, vertices are 
//	//specified in counter-clockwise order. 
//
//	osg::DrawElementsUInt* pyramidFaceOne =
//		new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
//	pyramidFaceOne->push_back(0);
//	pyramidFaceOne->push_back(1);
//	pyramidFaceOne->push_back(4);
//	pyramidGeometry->addPrimitiveSet(pyramidFaceOne);
//
//	osg::DrawElementsUInt* pyramidFaceTwo =
//		new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
//	pyramidFaceTwo->push_back(1);
//	pyramidFaceTwo->push_back(2);
//	pyramidFaceTwo->push_back(4);
//	pyramidGeometry->addPrimitiveSet(pyramidFaceTwo);
//
//	osg::DrawElementsUInt* pyramidFaceThree =
//		new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
//	pyramidFaceThree->push_back(2);
//	pyramidFaceThree->push_back(3);
//	pyramidFaceThree->push_back(4);
//	pyramidGeometry->addPrimitiveSet(pyramidFaceThree);
//
//	osg::DrawElementsUInt* pyramidFaceFour =
//		new osg::DrawElementsUInt(osg::PrimitiveSet::TRIANGLES, 0);
//	pyramidFaceFour->push_back(3);
//	pyramidFaceFour->push_back(0);
//	pyramidFaceFour->push_back(4);
//	pyramidGeometry->addPrimitiveSet(pyramidFaceFour);
//
//	//Declare and load an array of Vec4 elements to store colors. 
//
//	osg::Vec4Array* colors = new osg::Vec4Array;
//	colors->push_back(osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f)); //index 0 red
//	//colors->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f)); //index 1 green
//	//colors->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f)); //index 2 blue
//	//colors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f)); //index 3 white
//
//														  //Declare the variable that will match vertex array elements to color 
//														  //array elements. This vector should have the same number of elements 
//														  //as the number of vertices. This vector serves as a link between 
//														  //vertex arrays and color arrays. Entries in this index array 
//														  //coorespond to elements in the vertex array. Their values coorespond 
//														  //to the index in he color array. This same scheme would be followed 
//														  //if vertex array elements were matched with normal or texture 
//														  //coordinate arrays.
//														  //   Note that in this case, we are assigning 5 vertices to four 
//														  //   colors. Vertex array element zero (bottom left) and four (peak) 
//														  //   are both assigned to color array element zero (red).
//
//	osg::TemplateIndexArray
//		<unsigned int, osg::Array::UIntArrayType, 4, 4> *colorIndexArray;
//	colorIndexArray =
//		new osg::TemplateIndexArray<unsigned int, osg::Array::UIntArrayType, 4, 4>;
//	colorIndexArray->push_back(0); // vertex 0 assigned color array element 0
//	colorIndexArray->push_back(1); // vertex 1 assigned color array element 1
//	colorIndexArray->push_back(2); // vertex 2 assigned color array element 2
//	colorIndexArray->push_back(3); // vertex 3 assigned color array element 3
//	colorIndexArray->push_back(0); // vertex 4 assigned color array element 0
//
//								   //The next step is to associate the array of colors with the geometry, 
//								   //assign the color indices created above to the geometry and set the 
//								   //binding mode to _PER_VERTEX.
//
//	pyramidGeometry->setColorArray(colors);
//	//pyramidGeometry->setColorIndices(colorIndexArray);
//	//pyramidGeometry->setColorBinding(osg::Geometry::BIND_OVERALL);
//	pyramidGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
//	crossGeometry->setColorArray(colors, osg::Array::BIND_PER_PRIMITIVE_SET);
//	//crossGeometry->setColorIndices(colorIndexArray);
//	//crossGeometry->setColorBinding(osg::Geometry::BIND_PER_VERTEX);
//
//	//Now that we have created a geometry node and added it to the scene 
//	//we can reuse this geometry. For example, if we wanted to put a 
//	//second pyramid 15 units to the right of the first one, we could add 
//	//this geode as the child of a transform node in our scene graph. 
//
//	// Declare and initialize a transform node.
//	osg::PositionAttitudeTransform* pyramidTwoXForm =
//		new osg::PositionAttitudeTransform();
//
//	// Use the 'addChild' method of the osg::Group class to
//	// add the transform as a child of the root node and the
//	// pyramid node as a child of the transform.
//
//	root->addChild(pyramidTwoXForm);
//	pyramidTwoXForm->addChild(pyramidGeode);
//
//	// Declare and initialize a Vec3 instance to change the
//	// position of the model in the scene
//
//	osg::Vec3 pyramidTwoPosition(15, 0, 0);
//	pyramidTwoXForm->setPosition(pyramidTwoPosition);
//
//	//The final step is to set up and enter a simulation loop.
//
//	viewer.setSceneData(root);
//	viewer.setUpViewInWindow(40, 40, 600, 600);
//	//viewer.run();
//
//	osgGA::TrackballManipulator* _cameraManipulator = new osgGA::TrackballManipulator();
//	//viewer.setCameraManipulator(new osgGA::TrackballManipulator());
//	viewer.setCameraManipulator(_cameraManipulator);
//	_cameraManipulator->setDistance(100.0f);
//	_cameraManipulator->setRotation(osg::Quat(1, osg::Vec3d(1, 0.5, 0.5)));
//	//viewer.getCamera()->setClearColor(osg::Vec4(90 / 255.0, 102 / 255.0, 117 / 255.0, 1.0f));
//	viewer.getCamera()->setClearColor(osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
//	viewer.realize();
//
//	while (!viewer.done())
//	{
//		viewer.frame();
//	}
//
//	std::cout << "Program Complete" << std::endl;
//	_getch();
//	return 0;
//}
