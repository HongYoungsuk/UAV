#include "ModelSTL.h"

void ModelSTL::Load(string filename)
{
	stl_content = srSTL::readSTLfile(filename);
	//Draw();
}

void ModelSTL::Draw()
{
	//glPushMatrix();
	//glMultMatrixf(_T);
	for (srSTL::STLc::iterator point = stl_content.begin(); point != stl_content.end(); point++)
	{
		glBegin(GL_TRIANGLES);
		glNormal3f(point->normal[0], point->normal[1], point->normal[2]);
		for (int i = 0; i < 3; i++)
		{
			glVertex3f(point->vertex[i][0], point->vertex[i][1], point->vertex[i][2]);
		}
		glEnd();
	}
	//glPopMatrix();
}