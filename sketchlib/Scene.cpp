#include "Scene.h"

#include <geoshape/Point2D.h>
#include <geoshape/Line2D.h>
#include <geoshape/Circle.h>
#include <geoshape/Arc.h>
#include <geoshape/Ellipse.h>

namespace
{

bool is_none(const std::pair<sketchlib::GeoID, sketchlib::GeoType>& geo) {
    return geo.second == sketchlib::GeoType::None;
}

bool is_point(const std::pair<sketchlib::GeoID, sketchlib::GeoType>& geo) {
    return geo.second == sketchlib::GeoType::Point
        || geo.second == sketchlib::GeoType::GeoPtStart
        || geo.second == sketchlib::GeoType::GeoPtMid
        || geo.second == sketchlib::GeoType::GeoPtEnd;
}

bool is_line(const std::pair<sketchlib::GeoID, sketchlib::GeoType>& geo) {
    return geo.second == sketchlib::GeoType::Line;
}

bool is_circle(const std::pair<sketchlib::GeoID, sketchlib::GeoType>& geo) {
    return geo.second == sketchlib::GeoType::Circle;
}

bool is_arc(const std::pair<sketchlib::GeoID, sketchlib::GeoType>& geo) {
    return geo.second == sketchlib::GeoType::Arc;
}

bool is_ellipse(const std::pair<sketchlib::GeoID, sketchlib::GeoType>& geo) {
    return geo.second == sketchlib::GeoType::Ellipse;
}

sketchlib::PointPos get_point_pos(const std::pair<sketchlib::GeoID, sketchlib::GeoType>& geo) 
{
    auto pos = sketchlib::PointPos::None;

    if (geo.second == sketchlib::GeoType::Point) {
        pos = sketchlib::PointPos::Mid;
    } else if (geo.second == sketchlib::GeoType::GeoPtStart) {
        pos = sketchlib::PointPos::Start;
    } else if (geo.second == sketchlib::GeoType::GeoPtMid) {
        pos = sketchlib::PointPos::Mid;
    } else if (geo.second == sketchlib::GeoType::GeoPtEnd) {
        pos = sketchlib::PointPos::End;
    }

    return pos;
}

}

namespace sketchlib
{

Scene::Scene()
{
	m_gcs = std::make_shared<GCS::System>();
}

Scene::~Scene()
{
    for (auto& p : m_parameters) {
        delete p;
    }
}

void Scene::AddGeometry(GeoID id, const std::shared_ptr<gs::Shape2D>& shape)
{
    if (!shape || m_geoid2index.find(id) != m_geoid2index.end()) {
        return;
    }

    size_t idx = m_geos.size();
    switch (shape->GetType())
    {
    case gs::ShapeType2D::Point:
        AddPoint(std::static_pointer_cast<gs::Point2D>(shape), id);
        break;
    case gs::ShapeType2D::Line:
        AddLine(std::static_pointer_cast<gs::Line2D>(shape), id);
        break;
    case gs::ShapeType2D::Circle:
        AddCircle(std::static_pointer_cast<gs::Circle>(shape), id);
        break;
    case gs::ShapeType2D::Arc:
        AddArc(std::static_pointer_cast<gs::Arc>(shape), id);
        break;
    case gs::ShapeType2D::Ellipse:
        AddEllipse(std::static_pointer_cast<gs::Ellipse>(shape), id);
        break;
    }

    m_geoid2index[id] = idx;
}

void Scene::AddConstraint(ConsID id, ConsType type, const std::pair<GeoID, GeoType>& geo1,
                          const std::pair<GeoID, GeoType>& geo2, double val, bool driving)
{
    if (m_consid2index.find(id) != m_consid2index.end()) {
        return;
    }

    Constraint cons(id, type, geo1, geo2, val);

    size_t idx = m_cons.size();
    m_cons.push_back(cons);
    m_consid2index[id] = idx;

    double* cons_val = &m_cons.back().value;

    int geo1_idx = m_geoid2index[geo1.first];
    int geo2_idx = m_geoid2index[geo2.first];

    switch (type)
    {
        // basic
    case ConsType::Distance:
        if (is_point(geo1) && is_point(geo2)) {
            AddP2PDistanceCons(id, geo1_idx, get_point_pos(geo1), geo2_idx, get_point_pos(geo2), cons_val, driving);
        } else if (is_point(geo1) && is_line(geo2)) {
            AddP2LDistanceCons(id, geo1_idx, get_point_pos(geo1), geo2_idx, cons_val, driving);
        } else if (is_line(geo1) && is_point(geo2)) {
            AddP2LDistanceCons(id, geo2_idx, get_point_pos(geo2), geo1_idx, cons_val, driving);
        } else if (is_line(geo1) && is_none(geo2)) {
            AddP2PDistanceCons(id, geo1_idx, PointPos::Start, geo1_idx, PointPos::End, cons_val, driving);
        } else if (is_line(geo2) && is_none(geo1)) {
            AddP2PDistanceCons(id, geo2_idx, PointPos::Start, geo2_idx, PointPos::End, cons_val, driving);
        } else if (is_line(geo1) && is_line(geo2)) {
            AddP2PDistanceCons(id, geo1_idx, PointPos::Start, geo2_idx, PointPos::Start, cons_val, driving);
        }
        break;
    case ConsType::DistanceX:
        if (is_point(geo1) && is_point(geo2)) {
            AddDistanceXCons(id, geo1_idx, get_point_pos(geo1), geo2_idx, get_point_pos(geo2), cons_val, driving);
        } else if (is_line(geo1) && is_none(geo2)) {
            AddDistanceXCons(id, geo1_idx, PointPos::Start, geo1_idx, PointPos::End, cons_val, driving);
        } else if (is_line(geo2) && is_none(geo1)) {
            AddDistanceXCons(id, geo2_idx, PointPos::Start, geo2_idx, PointPos::End, cons_val, driving);
        } else if (is_point(geo1) && is_none(geo2)) {
            AddCoordinateXCons(id, geo1_idx, get_point_pos(geo1), cons_val, driving);
        } else if (is_point(geo2) && is_none(geo1)) {
            AddCoordinateXCons(id, geo2_idx, get_point_pos(geo2), cons_val, driving);
        }
        break;
    case ConsType::DistanceY:
        if (is_point(geo1) && is_point(geo2)) {
            AddDistanceYCons(id, geo1_idx, get_point_pos(geo1), geo2_idx, get_point_pos(geo2), cons_val, driving);
        } else if (is_line(geo1) && is_none(geo2)) {
            AddDistanceYCons(id, geo1_idx, PointPos::Start, geo1_idx, PointPos::End, cons_val, driving);
        } else if (is_line(geo2) && is_none(geo1)) {
            AddDistanceYCons(id, geo2_idx, PointPos::Start, geo2_idx, PointPos::End, cons_val, driving);
        } else if (is_point(geo1) && is_none(geo2)) {
            AddCoordinateYCons(id, geo1_idx, get_point_pos(geo1), cons_val, driving);
        } else if (is_point(geo2) && is_none(geo1)) {
            AddCoordinateYCons(id, geo2_idx, get_point_pos(geo2), cons_val, driving);
        }
        break;
    case ConsType::Angle:
        if (is_point(geo1) && is_point(geo2)) {
            AddP2PAngleCons(id, geo1_idx, get_point_pos(geo1), geo2_idx, get_point_pos(geo2), cons_val, driving);
        } else if (is_line(geo1) && is_none(geo2)) {
            AddP2PAngleCons(id, geo1_idx, PointPos::Start, geo1_idx, PointPos::End, cons_val, driving);
        } else if (is_line(geo2) && is_none(geo1)) {
            AddP2PAngleCons(id, geo2_idx, PointPos::Start, geo2_idx, PointPos::End, cons_val, driving);
        }
        break;
    case ConsType::Parallel:
        if (is_line(geo1) && is_line(geo2)) {
            AddParallelCons(id, geo1_idx, geo2_idx, driving);
        }
        break;
    case ConsType::Perpendicular:
        if (is_line(geo1) && is_line(geo2)) {
            AddPerpendicularCons(id, geo1_idx, geo2_idx, driving);
        }
        break;
    case ConsType::Coincident:
        if (is_point(geo1) && is_point(geo2)) {
            AddP2PCoincidentCons(id, geo1_idx, get_point_pos(geo1), geo2_idx, get_point_pos(geo2), driving);
        }
    case ConsType::Horizontal:
        if (is_point(geo1) && is_point(geo2)) {
            AddHorizontalCons(id, geo1_idx, get_point_pos(geo1), geo2_idx, get_point_pos(geo2), driving);
        } else if (is_line(geo1) && is_none(geo2)) {
            AddHorizontalCons(id, geo1_idx, PointPos::Start, geo1_idx, PointPos::End, driving);
        } else if (is_line(geo2) && is_none(geo1)) {
            AddHorizontalCons(id, geo2_idx, PointPos::Start, geo2_idx, PointPos::End, driving);
        }
        break;
    case ConsType::Vertical:
        if (is_point(geo1) && is_point(geo2)) {
            AddVerticalCons(id, geo1_idx, get_point_pos(geo1), geo2_idx, get_point_pos(geo2), driving);
        } else if (is_line(geo1) && is_none(geo2)) {
            AddVerticalCons(id, geo1_idx, PointPos::Start, geo1_idx, PointPos::End, driving);
        } else if (is_line(geo2) && is_none(geo1)) {
            AddVerticalCons(id, geo2_idx, PointPos::Start, geo2_idx, PointPos::End, driving);
        }
        break;
        // point on
    case ConsType::PointOnLine:
        if (is_point(geo1) && is_line(geo2)) {
            AddPointOnLineCons(id, geo1_idx, get_point_pos(geo1), geo2_idx, driving);
        } else if (is_point(geo2) && is_line(geo1)) {
            AddPointOnLineCons(id, geo2_idx, get_point_pos(geo2), geo1_idx, driving);
        }
        break;
    case ConsType::PointOnCircle:
        if (is_point(geo1) && is_circle(geo2)) {
            AddPointOnCircleCons(id, geo1_idx, get_point_pos(geo1), geo2_idx, driving);
        } else if (is_point(geo2) && is_circle(geo1)) {
            AddPointOnCircleCons(id, geo2_idx, get_point_pos(geo2), geo1_idx, driving);
        }
        break;
    case ConsType::PointOnArc:
        if (is_point(geo1) && is_arc(geo2)) {
            AddPointOnArcCons(id, geo1_idx, get_point_pos(geo1), geo2_idx, driving);
        } else if (is_point(geo2) && is_arc(geo1)) {
            AddPointOnArcCons(id, geo2_idx, get_point_pos(geo2), geo1_idx, driving);
        }
        break;
    case ConsType::PointOnEllipse:
        if (is_point(geo1) && is_ellipse(geo2)) {
            AddPointOnEllipseCons(id, geo1_idx, get_point_pos(geo1), geo2_idx, driving);
        } else if (is_point(geo2) && is_ellipse(geo1)) {
            AddPointOnEllipseCons(id, geo2_idx, get_point_pos(geo2), geo1_idx, driving);
        }
        break;
    case ConsType::PointOnPerpBisector:
        if (is_point(geo1) && is_line(geo2)) {
            AddPointOnPerpBisectorCons(id, geo1_idx, get_point_pos(geo1), geo2_idx, driving);
        } else if (is_point(geo2) && is_line(geo1)) {
            AddPointOnPerpBisectorCons(id, geo2_idx, get_point_pos(geo2), geo1_idx, driving);
        }
        break;
    case ConsType::MidpointOnLine:
        if (is_line(geo1) && is_line(geo2)) {
            AddMidpointOnLineCons(id, geo1_idx, geo2_idx, driving);
        }
        break;
        // tangent
    case ConsType::Tangent:
        if (is_line(geo1) && is_circle(geo2)) {
            AddL2CTangentCons(id, geo1_idx, geo2_idx, driving);
        } else if (is_line(geo2) && is_circle(geo1)) {
            AddL2CTangentCons(id, geo2_idx, geo1_idx, driving);
        } else if (is_circle(geo1) && is_circle(geo2)) {
            AddC2CTangentCons(id, geo1_idx, geo2_idx, driving);
        }
        break;
    case ConsType::TangentCircumf:
        if (is_circle(geo1) && is_circle(geo2)) {
            AddTangentCircumfCons(id, geo1_idx, geo2_idx, driving);
        }
        break;
        // params
    case ConsType::CircleRadius:
        if (is_circle(geo1)) {
            AddCircleRadiusCons(id, geo1_idx, cons_val, driving);
        } else if (is_circle(geo2)) {
            AddCircleRadiusCons(id, geo2_idx, cons_val, driving);
        }
        break;
    case ConsType::CircleDiameter:
        if (is_circle(geo1)) {
            AddCircleDiameterCons(id, geo1_idx, cons_val, driving);
        } else if (is_circle(geo2)) {
            AddCircleDiameterCons(id, geo2_idx, cons_val, driving);
        }
        break;
    case ConsType::ArcRadius:
        if (is_arc(geo1)) {
            AddArcRadiusCons(id, geo1_idx, cons_val, driving);
        } else if (is_arc(geo2)) {
            AddArcRadiusCons(id, geo2_idx, cons_val, driving);
        }
        break;
    case ConsType::ArcDiameter:
        if (is_arc(geo1)) {
            AddArcDiameterCons(id, geo1_idx, cons_val, driving);
        } else if (is_arc(geo2)) {
            AddArcDiameterCons(id, geo2_idx, cons_val, driving);
        }
        break;
    }
}
    
bool Scene::Solve(const std::vector<std::pair<GeoID, std::shared_ptr<gs::Shape2D>>>& geos)
{
    BeforeSolve(geos);

    int status = m_gcs->solve();
//    int status = m_gcs->solve(true, GCS::BFGS);
    if (status == GCS::Success) 
    {
        m_gcs->applySolution();
        AfterSolve(geos);

        return true;
    }
    else
    {
        return false;
    }
}

void Scene::Clear()
{
    m_gcs->clear();

    for (auto& p : m_parameters) {
        delete p;
    }
    m_parameters.clear();

    m_geoid2index.clear();
    m_geos.clear();

    m_points.clear();
    m_lines.clear();
    m_circles.clear();

    m_conflicting.clear();
    m_redundant.clear();
    m_partially_redundant.clear();
}

int Scene::GetDOF() const
{
    return m_gcs->dofsNumber();
}

void Scene::ResetSolver()
{
    m_gcs->declareUnknowns(m_parameters);
    m_gcs->initSolution(m_default_solver_redundant);

    m_gcs->getConflicting(m_conflicting);
    m_gcs->getRedundant(m_redundant);
    m_gcs->getPartiallyRedundant(m_partially_redundant);
}

void Scene::AddPoint(const std::shared_ptr<gs::Point2D>& pt, GeoID id)
{
    auto& pos = pt->GetPos();
    auto x = new double(pos.x);
    auto y = new double(pos.y);

    GCS::Point dst;
    dst.x = x;
    dst.y = y;

    m_parameters.push_back(x);
    m_parameters.push_back(y);

    int point_id = static_cast<int>(m_points.size());
    m_points.push_back(dst);

    Geometry geo;
    geo.id = id;
    geo.index = point_id;
    geo.start_pt_idx = point_id;
    geo.mid_pt_idx = point_id;
    geo.end_pt_idx = point_id;

    m_geos.push_back(geo);
}

void Scene::AddLine(const std::shared_ptr<gs::Line2D>& line, GeoID id)
{
    auto& s = line->GetStart();
    auto& e = line->GetEnd();
    GCS::Point p1, p2;

    m_parameters.push_back(new double(s.x));
    m_parameters.push_back(new double(s.y));
    p1.x = m_parameters[m_parameters.size() - 2];
    p1.y = m_parameters[m_parameters.size() - 1];

    m_parameters.push_back(new double(e.x));
    m_parameters.push_back(new double(e.y));
    p2.x = m_parameters[m_parameters.size() - 2];
    p2.y = m_parameters[m_parameters.size() - 1];

    Geometry geo;
    geo.id = id;
    geo.start_pt_idx = static_cast<int>(m_points.size());
    geo.end_pt_idx   = geo.start_pt_idx + 1;
    m_points.push_back(p1);
    m_points.push_back(p2);

    GCS::Line l;
    l.p1 = p1;
    l.p2 = p2;
    geo.index = m_lines.size();
    m_lines.push_back(l);

    m_geos.push_back(geo);
}

void Scene::AddCircle(const std::shared_ptr<gs::Circle>& circle, GeoID id)
{
    GCS::Point center;

    auto& pos = circle->GetCenter();
    m_parameters.push_back(new double(pos.x));
    m_parameters.push_back(new double(pos.y));
    center.x = m_parameters[m_parameters.size() - 2];
    center.y = m_parameters[m_parameters.size() - 1];

    float radius = circle->GetRadius();
    m_parameters.push_back(new double(radius));
    double* r = m_parameters[m_parameters.size() - 1];

    Geometry geo;
    geo.id = id;
    geo.mid_pt_idx = static_cast<int>(m_points.size());
    m_points.push_back(center);
        
    GCS::Circle c;
    c.center = center;
    c.rad    = r;
    geo.index = m_circles.size();
    m_circles.push_back(c);

    m_geos.push_back(geo);
}

void Scene::AddArc(const std::shared_ptr<gs::Arc>& arc, GeoID id)
{
    GCS::Point p1, p2, p3;

    auto& pos = arc->GetCenter();

    float radius = arc->GetRadius();

    float start_angle, end_angle;
    arc->GetAngles(start_angle, end_angle);

    sm::vec2 start_pos, end_pos;
    start_pos.x = pos.x + radius * std::cosf(start_angle);
    start_pos.y = pos.y + radius * std::sinf(start_angle);
    end_pos.x = pos.x + radius * std::cosf(end_angle);
    end_pos.y = pos.y + radius * std::sinf(end_angle);

    m_parameters.push_back(new double(start_pos.x));
    m_parameters.push_back(new double(start_pos.y));
    p1.x = m_parameters[m_parameters.size() - 2];
    p1.y = m_parameters[m_parameters.size() - 1];

    m_parameters.push_back(new double(end_pos.x));
    m_parameters.push_back(new double(end_pos.y));
    p2.x = m_parameters[m_parameters.size() - 2];
    p2.y = m_parameters[m_parameters.size() - 1];

    m_parameters.push_back(new double(pos.x));
    m_parameters.push_back(new double(pos.y));
    p3.x = m_parameters[m_parameters.size() - 2];
    p3.y = m_parameters[m_parameters.size() - 1];

    m_parameters.push_back(new double(radius));
    double* r = m_parameters[m_parameters.size() - 1];
    m_parameters.push_back(new double(start_angle));
    double* a1 = m_parameters[m_parameters.size() - 1];
    m_parameters.push_back(new double(end_angle));
    double* a2 = m_parameters[m_parameters.size() - 1];

    Geometry geo;
    geo.id = id;
    geo.start_pt_idx = static_cast<int>(m_points.size());
    m_points.push_back(p1);
    geo.end_pt_idx = static_cast<int>(m_points.size());
    m_points.push_back(p2);
    geo.mid_pt_idx = static_cast<int>(m_points.size());
    m_points.push_back(p3);

    GCS::Arc a;
    a.start  = p1;
    a.end    = p2;
    a.center = p3;
    a.rad    = r;
    a.startAngle = a1;
    a.endAngle   = a2;
    geo.index = m_arcs.size();
    m_arcs.push_back(a);

    m_geos.push_back(geo);
}

void Scene::AddEllipse(const std::shared_ptr<gs::Ellipse>& ellipse, GeoID id)
{
    auto& pos = ellipse->GetCenter();

    float radius_x, radius_y;
    ellipse->GetRadius(radius_x, radius_y);

    float dist_C_F = std::sqrtf(radius_x * radius_x - radius_y * radius_y);
    sm::vec2 focus1 = pos + sm::vec2(1, 0) * dist_C_F;  // radmajdir: (1, 0)

    GCS::Point center;

    m_parameters.push_back(new double(pos.x));
    m_parameters.push_back(new double(pos.y));
    center.x = m_parameters[m_parameters.size() - 2];
    center.y = m_parameters[m_parameters.size() - 1];

    m_parameters.push_back(new double(focus1.x));
    m_parameters.push_back(new double(focus1.y));
    double* f1x = m_parameters[m_parameters.size() - 2];
    double* f1y = m_parameters[m_parameters.size() - 1];

    m_parameters.push_back(new double(radius_y));
    double* rmin = m_parameters[m_parameters.size() - 1];

    Geometry geo;
    geo.id = id;

    GCS::Ellipse e;
    e.focus1.x = f1x;
    e.focus1.y = f1y;
    e.center = center;
    e.radmin = rmin;
    geo.index = m_ellipses.size();
    m_ellipses.push_back(e);

    m_geos.push_back(geo);
}

void Scene::AddP2PDistanceCons(ConsID id, int geo1, PointPos pos1, int geo2, PointPos pos2, double* value, bool driving)
{
    assert(geo1 < m_geos.size() && geo2 < m_geos.size());
    auto p1 = m_geos[geo1].GetPointID(pos1);
    auto p2 = m_geos[geo2].GetPointID(pos2);
    assert(p1 < m_points.size() && p2 < m_points.size());

    m_gcs->addConstraintP2PDistance(m_points[p1], m_points[p2], value, id, driving);

    ResetSolver();
}

void Scene::AddP2LDistanceCons(ConsID id, int pt_geo, PointPos pt_pos, int line, double* value, bool driving)
{
    assert(pt_geo < m_geos.size() && line < m_geos.size());
    auto p = m_geos[pt_geo].GetPointID(pt_pos);
    assert(p < m_points.size());

    m_gcs->addConstraintP2LDistance(m_points[p], m_lines[m_geos[line].index], value, id, driving);

    ResetSolver();
}

void Scene::AddDistanceXCons(ConsID id, int geo1, PointPos pos1, int geo2, PointPos pos2, double* value, bool driving)
{
    assert(geo1 < m_geos.size() && geo2 < m_geos.size());
    auto p1 = m_geos[geo1].GetPointID(pos1);
    auto p2 = m_geos[geo2].GetPointID(pos2);
    assert(p1 < m_points.size() && p2 < m_points.size());

    m_gcs->addConstraintDifference(m_points[p1].x, m_points[p2].x, value, id, driving);

    ResetSolver();
}

void Scene::AddDistanceYCons(ConsID id, int geo1, PointPos pos1, int geo2, PointPos pos2, double* value, bool driving)
{
    assert(geo1 < m_geos.size() && geo2 < m_geos.size());
    auto p1 = m_geos[geo1].GetPointID(pos1);
    auto p2 = m_geos[geo2].GetPointID(pos2);
    assert(p1 < m_points.size() && p2 < m_points.size());

    m_gcs->addConstraintDifference(m_points[p1].y, m_points[p2].y, value, id, driving);

    ResetSolver();
}

void Scene::AddCoordinateXCons(ConsID id, int geo, PointPos pos, double* value, bool driving)
{
    assert(geo < m_geos.size());
    auto p = m_geos[geo].GetPointID(pos);
    assert(p < m_points.size());

    m_gcs->addConstraintCoordinateX(m_points[p], value, id, driving);

    ResetSolver();
}

void Scene::AddCoordinateYCons(ConsID id, int geo, PointPos pos, double* value, bool driving)
{
    assert(geo < m_geos.size());
    auto p = m_geos[geo].GetPointID(pos);
    assert(p < m_points.size());

    m_gcs->addConstraintCoordinateY(m_points[p], value, id, driving);

    ResetSolver();
}

void Scene::AddP2PAngleCons(ConsID id, int geo1, PointPos pos1, int geo2, PointPos pos2, double* value, bool driving)
{
    assert(geo1 < m_geos.size() && geo2 < m_geos.size());
    auto p1 = m_geos[geo1].GetPointID(pos1);
    auto p2 = m_geos[geo2].GetPointID(pos2);
    assert(p1 < m_points.size() && p2 < m_points.size());

    m_gcs->addConstraintP2PAngle(m_points[p1], m_points[p2], value, id, driving);

    ResetSolver();
}

void Scene::AddParallelCons(ConsID id, int line1, int line2, bool driving)
{
    assert(line1 < m_geos.size() && line2 < m_geos.size());

    m_gcs->addConstraintParallel(m_lines[m_geos[line1].index], m_lines[m_geos[line2].index], id, driving);

    ResetSolver();
}

void Scene::AddPerpendicularCons(ConsID id, int line1, int line2, bool driving)
{
    assert(line1 < m_geos.size() && line2 < m_geos.size());

    m_gcs->addConstraintPerpendicular(m_lines[m_geos[line1].index], m_lines[m_geos[line2].index], id, driving);

    ResetSolver();
}

void Scene::AddP2PCoincidentCons(ConsID id, int geo1, PointPos pos1, int geo2, PointPos pos2, bool driving)
{
    assert(geo1 < m_geos.size() && geo2 < m_geos.size());
    auto p1 = m_geos[geo1].GetPointID(pos1);
    auto p2 = m_geos[geo2].GetPointID(pos2);
    assert(p1 < m_points.size() && p2 < m_points.size());

    m_gcs->addConstraintP2PCoincident(m_points[p1], m_points[p2], id, driving);

    ResetSolver();
}

void Scene::AddHorizontalCons(ConsID id, int geo1, PointPos pos1, int geo2, PointPos pos2, bool driving)
{
    assert(geo1 < m_geos.size() && geo2 < m_geos.size());
    auto p1 = m_geos[geo1].GetPointID(pos1);
    auto p2 = m_geos[geo2].GetPointID(pos2);
    assert(p1 < m_points.size() && p2 < m_points.size());

    m_gcs->addConstraintHorizontal(m_points[p1], m_points[p2], id, driving);

    ResetSolver();
}

void Scene::AddVerticalCons(ConsID id, int geo1, PointPos pos1, int geo2, PointPos pos2, bool driving)
{
    assert(geo1 < m_geos.size() && geo2 < m_geos.size());
    auto p1 = m_geos[geo1].GetPointID(pos1);
    auto p2 = m_geos[geo2].GetPointID(pos2);
    assert(p1 < m_points.size() && p2 < m_points.size());

    m_gcs->addConstraintVertical(m_points[p1], m_points[p2], id, driving);

    ResetSolver();
}

void Scene::AddPointOnLineCons(ConsID id, int pt_geo, PointPos pt_pos, int line, bool driving)
{
    assert(pt_geo < m_geos.size() && line < m_geos.size());
    auto p = m_geos[pt_geo].GetPointID(pt_pos);
    assert(p < m_points.size());

    m_gcs->addConstraintPointOnLine(m_points[p], m_lines[m_geos[line].index], id, driving);

    ResetSolver();
}

void Scene::AddPointOnCircleCons(ConsID id, int pt_geo, PointPos pt_pos, int circle, bool driving)
{
    assert(pt_geo < m_geos.size() && circle < m_geos.size());
    auto p = m_geos[pt_geo].GetPointID(pt_pos);
    assert(p < m_points.size());

    m_gcs->addConstraintPointOnCircle(m_points[p], m_circles[m_geos[circle].index], id, driving);

    ResetSolver();
}

void Scene::AddPointOnArcCons(ConsID id, int pt_geo, PointPos pt_pos, int arc, bool driving)
{
    assert(pt_geo < m_geos.size() && arc < m_geos.size());
    auto p = m_geos[pt_geo].GetPointID(pt_pos);
    assert(p < m_points.size());

    m_gcs->addConstraintPointOnArc(m_points[p], m_arcs[m_geos[arc].index], id, driving);

    ResetSolver();
}

void Scene::AddPointOnEllipseCons(ConsID id, int pt_geo, PointPos pt_pos, int ellipse, bool driving)
{
    assert(pt_geo < m_geos.size() && ellipse < m_geos.size());
    auto p = m_geos[pt_geo].GetPointID(pt_pos);
    assert(p < m_points.size());

    m_gcs->addConstraintPointOnEllipse(m_points[p], m_ellipses[m_geos[ellipse].index], id, driving);

    ResetSolver();
}

void Scene::AddPointOnPerpBisectorCons(ConsID id, int pt_geo, PointPos pt_pos, int line, bool driving)
{
    assert(pt_geo < m_geos.size() && line < m_geos.size());
    auto p = m_geos[pt_geo].GetPointID(pt_pos);
    assert(p < m_points.size());

    m_gcs->addConstraintPointOnPerpBisector(m_points[p], m_lines[m_geos[line].index], id, driving);

    ResetSolver();
}

void Scene::AddMidpointOnLineCons(ConsID id, int line1, int line2, bool driving)
{
    assert(line1 < m_geos.size() && line2 < m_geos.size());

    m_gcs->addConstraintMidpointOnLine(m_lines[m_geos[line1].index], m_lines[m_geos[line2].index], id, driving);

    ResetSolver();
}

void Scene::AddL2CTangentCons(ConsID id, int line, int circle, bool driving)
{
    assert(line < m_geos.size() && circle < m_geos.size());

    m_gcs->addConstraintTangent(m_lines[m_geos[line].index], m_circles[m_geos[circle].index], id, driving);

    ResetSolver();
}

void Scene::AddC2CTangentCons(ConsID id, int circle1, int circle2, bool driving)
{
    assert(circle1 < m_geos.size() && circle2 < m_geos.size());

    m_gcs->addConstraintTangent(m_circles[m_geos[circle1].index], m_circles[m_geos[circle2].index], id, driving);

    ResetSolver();
}

void Scene::AddL2ATangentCons(ConsID id, int line, int arc, bool driving)
{
    assert(line < m_geos.size() && arc < m_geos.size());

    auto& l = m_lines[m_geos[line].index];
    m_gcs->addConstraintPerpendicularLine2Arc(l.p1, l.p2, m_arcs[m_geos[arc].index], id, driving);

    ResetSolver();
}

void Scene::AddA2LTangentCons(ConsID id, int arc, int line, bool driving)
{
    assert(line < m_geos.size() && arc < m_geos.size());

    auto& l = m_lines[m_geos[line].index];
    m_gcs->addConstraintPerpendicularArc2Line(m_arcs[m_geos[arc].index], l.p1, l.p2, id, driving);

    ResetSolver();
}

void Scene::AddC2ATangentCons(ConsID id, int circle, int arc, bool driving)
{
    assert(circle < m_geos.size() && arc < m_geos.size());

    auto& c = m_circles[m_geos[circle].index];
    m_gcs->addConstraintPerpendicularCircle2Arc(c.center, c.rad, m_arcs[m_geos[arc].index], id, driving);

    ResetSolver();
}

void Scene::AddA2CTangentCons(ConsID id, int arc, int circle, bool driving)
{
    assert(circle < m_geos.size() && arc < m_geos.size());

    auto& c = m_circles[m_geos[circle].index];
    m_gcs->addConstraintPerpendicularArc2Circle(m_arcs[m_geos[arc].index], c.center, c.rad, id, driving);

    ResetSolver();
}

void Scene::AddA2ATangentCons(ConsID id, int arc1, int arc2, bool driving)
{
    assert(arc1 < m_geos.size() && arc2 < m_geos.size());

    m_gcs->addConstraintPerpendicularArc2Arc(m_arcs[m_geos[arc1].index], false, m_arcs[m_geos[arc2].index], false, id, driving);

    ResetSolver();
}

void Scene::AddTangentCircumfCons(ConsID id, int circle1, int circle2, bool driving)
{
    assert(circle1 < m_geos.size() && circle2 < m_geos.size());

    m_gcs->addConstraintTangentCircumf(m_points[m_geos[circle1].mid_pt_idx], m_points[m_geos[circle2].mid_pt_idx], 
        m_circles[m_geos[circle1].index].rad, m_circles[m_geos[circle2].index].rad, false, id, driving);

    ResetSolver();
}

void Scene::AddCircleRadiusCons(ConsID id, int circle, double* value, bool driving)
{
    assert(circle < m_geos.size());

    m_gcs->addConstraintCircleRadius(m_circles[m_geos[circle].index], value, id, driving);

    ResetSolver();
}

void Scene::AddCircleDiameterCons(ConsID id, int circle, double* value, bool driving)
{
    assert(circle < m_geos.size());

    m_gcs->addConstraintCircleDiameter(m_circles[m_geos[circle].index], value, id, driving);

    ResetSolver();

}

void Scene::AddArcRadiusCons(ConsID id, int arc, double* value, bool driving)
{
    assert(arc < m_geos.size());

    m_gcs->addConstraintArcRadius(m_arcs[m_geos[arc].index], value, id, driving);

    ResetSolver();
}

void Scene::AddArcDiameterCons(ConsID id, int arc, double* value, bool driving)
{
    assert(arc < m_geos.size());

    m_gcs->addConstraintArcDiameter(m_arcs[m_geos[arc].index], value, id, driving);

    ResetSolver();
}

void Scene::BeforeSolve(const std::vector<std::pair<GeoID, std::shared_ptr<gs::Shape2D>>>& geos)
{
    bool dirty = false;

    for (auto& geo : geos)
    {
        auto id = geo.first;
        auto itr = m_geoid2index.find(id);
        if (itr == m_geoid2index.end()) {
            continue;
        }

        auto index = m_geos[itr->second].index;

        auto shape = geo.second;
        auto type = shape->GetType();
        if (type == gs::ShapeType2D::Point)
        {
            auto src = std::static_pointer_cast<gs::Point2D>(shape);
            auto src_x = static_cast<double>(src->GetPos().x);
            auto src_y = static_cast<double>(src->GetPos().y);

            auto& dst = m_points[index];
            if (*dst.x != src_x || *dst.y != src_y) 
            {
                *dst.x = src_x;
                *dst.y = src_y;
                dirty = true;
            }
        }
        else if (type == gs::ShapeType2D::Line)
        {
            auto src = std::static_pointer_cast<gs::Line2D>(shape);
            auto p1_x = static_cast<double>(src->GetStart().x);
            auto p1_y = static_cast<double>(src->GetStart().y);
            auto p2_x = static_cast<double>(src->GetEnd().x);
            auto p2_y = static_cast<double>(src->GetEnd().y);

            auto& dst = m_lines[index];
            if (*dst.p1.x != p1_x || *dst.p1.y != p1_y ||
                *dst.p2.x != p2_x || *dst.p2.y != p2_y) 
            {
                *dst.p1.x = p1_x;
                *dst.p1.y = p1_y;
                *dst.p2.x = p2_x;
                *dst.p2.y = p2_y;
                dirty = true;
            }
        }
        else if (type == gs::ShapeType2D::Circle)
        {
            auto src = std::static_pointer_cast<gs::Circle>(shape);
            auto cx = static_cast<double>(src->GetCenter().x);
            auto cy = static_cast<double>(src->GetCenter().y);
            auto r = static_cast<double>(src->GetRadius());

            auto& dst = m_circles[index];
            if (*dst.center.x != cx || *dst.center.y != cy || *dst.rad != r) 
            {
                *dst.center.x = cx;
                *dst.center.y = cy;
                *dst.rad = r;
                dirty = true;
            }
        }
        else if (type == gs::ShapeType2D::Arc)
        {
            auto src = std::static_pointer_cast<gs::Arc>(shape);

            auto cx = static_cast<double>(src->GetCenter().x);
            auto cy = static_cast<double>(src->GetCenter().y);
            auto r = static_cast<double>(src->GetRadius());

            float sa, ea;
            src->GetAngles(sa, ea);
            auto start_angle = static_cast<double>(sa);
            auto end_angle = static_cast<double>(ea);

            auto& dst = m_arcs[index];
            if (*dst.center.x != cx || *dst.center.y != cy || *dst.rad != r || 
                *dst.startAngle != start_angle || *dst.endAngle != end_angle)
            {
                *dst.center.x = cx;
                *dst.center.y = cy;
                *dst.rad = r;
                *dst.startAngle = start_angle;
                *dst.endAngle = end_angle;
                dirty = true;
            }
        }
        else if (type == gs::ShapeType2D::Ellipse)
        {
            auto src = std::static_pointer_cast<gs::Ellipse>(shape);

            auto& pos = src->GetCenter();

            float radius_x, radius_y;
            src->GetRadius(radius_x, radius_y);

            float dist_C_F = std::sqrtf(radius_x * radius_x - radius_y * radius_y);
            sm::vec2 focus1 = pos + sm::vec2(1, 0) * dist_C_F;  // radmajdir: (1, 0)

            double f1x = static_cast<double>(focus1.x);
            double f1y = static_cast<double>(focus1.y);
            double cx = static_cast<double>(pos.x);
            double cy = static_cast<double>(pos.y);
            double rmin = static_cast<double>(radius_y);

            auto& dst = m_ellipses[index];
            if (*dst.center.x != cx || *dst.center.y != cy || *dst.focus1.x != f1x || 
                *dst.focus1.y != f1y || *dst.radmin != rmin)
            {
                *dst.center.x = cx;
                *dst.center.y = cy;
                *dst.focus1.x = f1x;
                *dst.focus1.y = f1y;
                *dst.radmin = rmin;
                dirty = true;
            }
        }
    }

    if (dirty) {
        m_gcs->initSolution(m_default_solver_redundant);
    }
}

void Scene::AfterSolve(const std::vector<std::pair<GeoID, std::shared_ptr<gs::Shape2D>>>& geos)
{
    bool dirty = false;

    for (auto& geo : geos)
    {
        auto id = geo.first;
        auto itr = m_geoid2index.find(id);
        if (itr == m_geoid2index.end()) {
            continue;
        }

        auto index = m_geos[itr->second].index;

        auto shape = geo.second;
        auto type = shape->GetType();
        if (type == gs::ShapeType2D::Point)
        {
            auto& src = m_points[index];
            auto dst = std::static_pointer_cast<gs::Point2D>(shape);

            sm::vec2 pos(static_cast<float>(*src.x), static_cast<float>(*src.y));
            if (pos != dst->GetPos()) 
            {
                dst->SetPos(pos);
                dirty = true;
            }
        }
        else if (type == gs::ShapeType2D::Line)
        {
            auto& src = m_lines[index];
            auto dst = std::static_pointer_cast<gs::Line2D>(shape);

            sm::vec2 p1(static_cast<float>(*src.p1.x), static_cast<float>(*src.p1.y));
            sm::vec2 p2(static_cast<float>(*src.p2.x), static_cast<float>(*src.p2.y));
            if (p1 != dst->GetStart() || p2 != dst->GetEnd()) 
            {
                dst->SetStart(p1);
                dst->SetEnd(p2);
                dirty = true;
            }
        }
        else if (type == gs::ShapeType2D::Circle)
        {
            auto& src = m_circles[index];
            auto dst = std::static_pointer_cast<gs::Circle>(shape);

            sm::vec2 c(static_cast<float>(*src.center.x), static_cast<float>(*src.center.y));
            float d = static_cast<float>(*src.rad);
            if (c != dst->GetCenter() || d != dst->GetRadius())
            {
                dst->SetCenter(c);
                dst->SetRadius(d);
                dirty = true;
            }
        }
        else if (type == gs::ShapeType2D::Arc)
        {
            auto& src = m_arcs[index];
            auto dst = std::static_pointer_cast<gs::Arc>(shape);

            sm::vec2 c(static_cast<float>(*src.center.x), static_cast<float>(*src.center.y));
            float d = static_cast<float>(*src.rad);

            sm::vec2 s(static_cast<float>(*src.start.x), static_cast<float>(*src.start.y));
            sm::vec2 e(static_cast<float>(*src.end.x), static_cast<float>(*src.end.y));

            //float start_angle = static_cast<float>(*src.startAngle);
            //float end_angle = static_cast<float>(*src.endAngle);

            float start_angle = std::atan2f(s.y - c.y, s.x - c.x);
            float end_angle = std::atan2f(e.y - c.y, e.x - c.x);
            if (end_angle < start_angle) {
                end_angle += SM_TWO_PI;
            }

            float sa, ea;
            dst->GetAngles(sa, ea);
            if (c != dst->GetCenter() || d != dst->GetRadius() ||
                start_angle != sa || end_angle != ea)
            {
                dst->SetCenter(c);
                dst->SetRadius(d);
                dst->SetAngles(start_angle, end_angle);
                dirty = true;
            }
        }
        else if (type == gs::ShapeType2D::Ellipse)
        {
            auto& src = m_ellipses[index];
            auto dst = std::static_pointer_cast<gs::Ellipse>(shape);

            sm::vec2 c(static_cast<float>(*src.center.x), static_cast<float>(*src.center.y));

            sm::vec2 r;
            r.y = static_cast<float>(*src.radmin);
            r.x = std::sqrtf(std::powf(*src.focus1.x - c.x, 2) + r.y * r.y);

            float rx, ry;
            dst->GetRadius(rx, ry);

            if (c != dst->GetCenter() || r.x != rx || r.y != ry)
            {
                dst->SetCenter(c);
                dst->SetRadius(r.x, r.y);
                dirty = true;
            }
        }
    }
}

}