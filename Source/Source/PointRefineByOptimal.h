#pragma once

#include <nlopt.hpp>

#include "TreeBase.h"
#include "HorizontalPartition.h"

/*

Refine stem points using optimal 

*/

class CPointRefineByOptimal : public CTreeBase
{
	Q_OBJECT
private:
	CHorizontalPartition HorizontalPartition;
	double ThickNess; //refine by the half of the seted thickNess 
	double Angle;

public:
	

};

