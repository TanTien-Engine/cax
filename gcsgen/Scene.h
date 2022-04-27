#pragma once

#include "Geometry.h"
#include "Constraint.h"
#include "../thirdparty/PlaneGCS/GCS.h"

#include <memory>

namespace gs { class Shape2D; class Point2D; class Line2D; class Circle; }

namespace gcsgen
{

class Scene
{
public:
	Scene();
	~Scene();

	void AddConstraint(const Constraint& cons);

	bool Solve();

	void Clear();

private:
	void ClearConstraints();

	void ResetSolver();

	// geometries
	int AddGeometry(const std::shared_ptr<gs::Shape2D>& geo);
	int AddPoint(const std::shared_ptr<gs::Point2D>& pt);
	int AddLine(const std::shared_ptr<gs::Line2D>& line);
	int AddCircle(const std::shared_ptr<gs::Circle>& circle);

	// basic constraints
	int AddP2PDistanceCons(int geo1, PointPos pos1, int geo2, PointPos pos2, double* value);
	int AddP2LDistanceCons(int point, int line, double* value);
	int AddP2PAngleCons(int geo1, PointPos pos1, int geo2, PointPos pos2, double* value);
	int AddParallelCons(int line1, int line2);
	int AddPerpendicularCons(int line1, int line2);
	int AddPointOnLineCons(int point, int line);
	int AddPointOnPerpBisectorCons(int point, int line);
	int AddMidpointOnLineCons(int line1, int line2);
	int AddTangentCircumfCons(int circle1, int circle2);

	// derived constraints
	int AddP2PCoincidentCons(int geo1, PointPos pos1, int geo2, PointPos pos2);
	int AddHorizontalCons(int geo1, PointPos pos1, int geo2, PointPos pos2);
	int AddVerticalCons(int geo1, PointPos pos1, int geo2, PointPos pos2);
	int AddPointOnCircleCons(int point, int circle);
	int AddL2CTangentCons(int line, int circle);
	int AddC2CTangentCons(int circle1, int circle2);

	void BeforeSolve();
	bool AfterSolve();

private:
	std::shared_ptr<GCS::System> m_gcs = nullptr;

	std::vector<double*> m_parameters;

	std::map<std::shared_ptr<gs::Shape2D>, int> m_geo_id;
	std::vector<Geometry> m_geos;

	GCS::Algorithm m_default_solver = GCS::DogLeg;
	GCS::Algorithm m_default_solver_redundant = GCS::DogLeg;

	std::vector<GCS::Point>  m_points;
	std::vector<GCS::Line>   m_lines;
	std::vector<GCS::Circle> m_circles;

	int m_constraints_counter = 0;

}; // Scene

}