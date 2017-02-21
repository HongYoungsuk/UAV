#ifndef __MODELSTL__
#define __MODELSTL__

#include <stdlib.h>
#include <gl/glut.h>
#include <gl/gl.h>
#include <gl/glu.h>
#include "common/srSTL.h"

class ModelSTL
{
private:
	srSTL::STLc stl_content;

public:
	float _T[16];

	void Load(string);
	void Draw();
};

#endif