#pragma once

#include "Geometry.h"
#include "Constraint.h"
#include "../thirdparty/PlaneGCS/GCS.h"

#include <memory>

namespace gs { class Shape2D; class Point2D; class Line2D; class Circle; class Arc; class Ellipse; }

namespace sketchlib
{

class Scene
{
public:
	Scene();
	~Scene();

	void AddGeometry(GeoID id, const std::shared_ptr<gs::Shape2D>& shape);
	void AddConstraint(ConsID id, ConsType type, const std::pair<GeoID, GeoType>& geo1, 
		const std::pair<GeoID, GeoType>& geo2, double val);

	bool Solve(const std::vector<std::pair<GeoID, std::shared_ptr<gs::Shape2D>>>& geos);

	void Clear();

private:
	void ResetSolver();

	// geometries
	void AddPoint(const std::shared_ptr<gs::Point2D>& pt, GeoID id);
	void AddLine(const std::shared_ptr<gs::Line2D>& line, GeoID id);
	void AddCircle(const std::shared_ptr<gs::Circle>& circle, GeoID id);
	void AddArc(const std::shared_ptr<gs::Arc>& arc, GeoID id);
	void AddEllipse(const std::shared_ptr<gs::Ellipse>& ellipse, GeoID id);

	// basic
	void AddP2PDistanceCons(ConsID id, int geo1, PointPos pos1, int geo2, PointPos pos2, double* value);
	void AddP2LDistanceCons(ConsID id, int point, int line, double* value);
	void AddP2PAngleCons(ConsID id, int geo1, PointPos pos1, int geo2, PointPos pos2, double* value);
	void AddParallelCons(ConsID id, int line1, int line2);
	void AddPerpendicularCons(ConsID id, int line1, int line2);
	void AddP2PCoincidentCons(ConsID id, int geo1, PointPos pos1, int geo2, PointPos pos2);
	void AddHorizontalCons(ConsID id, int geo1, PointPos pos1, int geo2, PointPos pos2);
	void AddVerticalCons(ConsID id, int geo1, PointPos pos1, int geo2, PointPos pos2);

	// point on
	void AddPointOnLineCons(ConsID id, int point, int line);
	void AddPointOnCircleCons(ConsID id, int point, int circle);
	void AddPointOnArcCons(ConsID id, int point, int arc);
	void AddPointOnEllipseCons(ConsID id, int point, int ellipse);
	void AddPointOnPerpBisectorCons(ConsID id, int point, int line);
	void AddMidpointOnLineCons(ConsID id, int line1, int line2);

	// tangent
	void AddL2CTangentCons(ConsID id, int line, int circle);
	void AddC2CTangentCons(ConsID id, int circle1, int circle2);
	void AddL2ATangentCons(ConsID id, int line, int arc);
	void AddA2LTangentCons(ConsID id, int arc, int line);
	void AddC2ATangentCons(ConsID id, int circle, int arc);
	void AddA2CTangentCons(ConsID id, int arc, int circle);
	void AddA2ATangentCons(ConsID id, int arc1, int arc2);

	void AddTangentCircumfCons(ConsID id, int circle1, int circle2);

	void BeforeSolve(const std::vector<std::pair<GeoID, std::shared_ptr<gs::Shape2D>>>& geos);
	void AfterSolve(const std::vector<std::pair<GeoID, std::shared_ptr<gs::Shape2D>>>& geos);

private:
	std::shared_ptr<GCS::System> m_gcs = nullptr;

	std::vector<double*> m_parameters;

	// geo
	std::vector<Geometry>   m_geos;
	std::map<GeoID, size_t> m_geoid2index;

	// cons
	std::vector<Constraint>  m_cons;
	std::map<ConsID, size_t> m_consid2index;

	GCS::Algorithm m_default_solver = GCS::DogLeg;
	GCS::Algorithm m_default_solver_redundant = GCS::DogLeg;

	std::vector<GCS::Point>   m_points;
	std::vector<GCS::Line>    m_lines;
	std::vector<GCS::Circle>  m_circles;
	std::vector<GCS::Arc>     m_arcs;
	std::vector<GCS::Ellipse> m_ellipses;

	std::vector<int> m_conflicting;
	std::vector<int> m_redundant;
	std::vector<int> m_partially_redundant;

}; // Scene

}