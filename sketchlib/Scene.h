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
		const std::pair<GeoID, GeoType>& geo2, double val, bool driving);

	bool Solve(const std::vector<std::pair<GeoID, std::shared_ptr<gs::Shape2D>>>& geos);

	void Clear();

	int GetDOF() const;

private:
	void ResetSolver();

	bool IsConsExists(ConsType type, const std::pair<GeoID, GeoType>& geo1,
		const std::pair<GeoID, GeoType>& geo2) const;

	// geometries
	void AddPoint(const std::shared_ptr<gs::Point2D>& pt, GeoID id);
	void AddLine(const std::shared_ptr<gs::Line2D>& line, GeoID id);
	void AddCircle(const std::shared_ptr<gs::Circle>& circle, GeoID id);
	void AddArc(const std::shared_ptr<gs::Arc>& arc, GeoID id);
	void AddEllipse(const std::shared_ptr<gs::Ellipse>& ellipse, GeoID id);

	// basic
	void AddP2PDistanceCons(ConsID id, int geo1, PointPos pos1, int geo2, PointPos pos2, double* value, bool driving);
	void AddP2LDistanceCons(ConsID id, int pt_geo, PointPos pt_pos, int line, double* value, bool driving);
	void AddDistanceXCons(ConsID id, int geo1, PointPos pos1, int geo2, PointPos pos2, double* value, bool driving);
	void AddDistanceYCons(ConsID id, int geo1, PointPos pos1, int geo2, PointPos pos2, double* value, bool driving);
	void AddCoordinateXCons(ConsID id, int geo, PointPos pos, double* value, bool driving);
	void AddCoordinateYCons(ConsID id, int geo, PointPos pos, double* value, bool driving);
	void AddP2PAngleCons(ConsID id, int geo1, PointPos pos1, int geo2, PointPos pos2, double* value, bool driving);
	void AddParallelCons(ConsID id, int line1, int line2, bool driving);
	void AddPerpendicularCons(ConsID id, int line1, int line2, bool driving);
	void AddP2PCoincidentCons(ConsID id, int geo1, PointPos pos1, int geo2, PointPos pos2, bool driving);
	void AddHorizontalCons(ConsID id, int geo1, PointPos pos1, int geo2, PointPos pos2, bool driving);
	void AddVerticalCons(ConsID id, int geo1, PointPos pos1, int geo2, PointPos pos2, bool driving);

	// point on
	void AddPointOnLineCons(ConsID id, int pt_geo, PointPos pt_pos, int line, bool driving);
	void AddPointOnCircleCons(ConsID id, int pt_geo, PointPos pt_pos, int circle, bool driving);
	void AddPointOnArcCons(ConsID id, int pt_geo, PointPos pt_pos, int arc, bool driving);
	void AddPointOnEllipseCons(ConsID id, int pt_geo, PointPos pt_pos, int ellipse, bool driving);
	void AddPointOnPerpBisectorCons(ConsID id, int pt_geo, PointPos pt_pos, int line, bool driving);
	void AddMidpointOnLineCons(ConsID id, int line1, int line2, bool driving);

	// tangent
	void AddL2CTangentCons(ConsID id, int line, int circle, bool driving);
	void AddC2CTangentCons(ConsID id, int circle1, int circle2, bool driving);
	void AddL2ATangentCons(ConsID id, int line, int arc, bool driving);
	void AddA2LTangentCons(ConsID id, int arc, int line, bool driving);
	void AddC2ATangentCons(ConsID id, int circle, int arc, bool driving);
	void AddA2CTangentCons(ConsID id, int arc, int circle, bool driving);
	void AddA2ATangentCons(ConsID id, int arc1, int arc2, bool driving);
	void AddTangentCircumfCons(ConsID id, int circle1, int circle2, bool driving);

	// params
	void AddCircleRadiusCons(ConsID id, int circle, double* value, bool driving);
	void AddCircleDiameterCons(ConsID id, int circle, double* value, bool driving);
	void AddArcRadiusCons(ConsID id, int arc, double* value, bool driving);
	void AddArcDiameterCons(ConsID id, int arc, double* value, bool driving);

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