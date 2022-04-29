#include "Scene.h"

#include <geoshape/Point2D.h>
#include <geoshape/Line2D.h>
#include <geoshape/Circle.h>

namespace
{

bool is_none(const std::pair<sketchlib::GeoID, sketchlib::GeoType>& geo) {
    return geo.second == sketchlib::GeoType::None;
}

bool is_point(const std::pair<sketchlib::GeoID, sketchlib::GeoType>& geo) {
    return geo.second == sketchlib::GeoType::Point;
}

bool is_line(const std::pair<sketchlib::GeoID, sketchlib::GeoType>& geo) {
    return geo.second == sketchlib::GeoType::Line;
}

bool is_circle(const std::pair<sketchlib::GeoID, sketchlib::GeoType>& geo) {
    return geo.second == sketchlib::GeoType::Circle;
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
    }

    m_geoid2index[id] = idx;
}

void Scene::AddConstraint(ConsID id, ConsType type, const std::pair<GeoID, GeoType>& geo1,
                          const std::pair<GeoID, GeoType>& geo2, double val)
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
            AddP2PDistanceCons(id, geo1_idx, PointPos::Mid, geo2_idx, PointPos::Mid, cons_val);
        } else if (is_point(geo1) && is_line(geo2)) {
            AddP2LDistanceCons(id, geo1_idx, geo2_idx, cons_val);
        } else if (is_line(geo1) && is_point(geo2)) {
            AddP2LDistanceCons(id, geo2_idx, geo1_idx, cons_val);
        } else if (is_line(geo1) && is_none(geo2)) {
            AddP2PDistanceCons(id, geo1_idx, PointPos::Start, geo1_idx, PointPos::End, cons_val);
        } else if (is_line(geo2) && is_none(geo1)) {
            AddP2PDistanceCons(id, geo2_idx, PointPos::Start, geo2_idx, PointPos::End, cons_val);
        } else if (is_line(geo1) && is_line(geo2)) {
            AddP2PDistanceCons(id, geo1_idx, PointPos::Start, geo2_idx, PointPos::Start, cons_val);
        }
        break;
    case ConsType::Angle:
        if (is_point(geo1) && is_point(geo2)) {
            AddP2PAngleCons(id, geo1_idx, PointPos::Mid, geo2_idx, PointPos::Mid, cons_val);
        } else if (is_line(geo1) && is_none(geo2)) {
            AddP2PAngleCons(id, geo1_idx, PointPos::Start, geo1_idx, PointPos::End, cons_val);
        } else if (is_line(geo2) && is_none(geo1)) {
            AddP2PAngleCons(id, geo2_idx, PointPos::Start, geo2_idx, PointPos::End, cons_val);
        }
        break;
    case ConsType::Parallel:
        if (is_line(geo1) && is_line(geo2)) {
            AddParallelCons(id, geo1_idx, geo2_idx);
        }
        break;
    case ConsType::Perpendicular:
        if (is_line(geo1) && is_line(geo2)) {
            AddPerpendicularCons(id, geo1_idx, geo2_idx);
        }
        break;
    case ConsType::PointOnLine:
        if (is_point(geo1) && is_line(geo2)) {
            AddPointOnLineCons(id, geo1_idx, geo2_idx);
        } else if (is_point(geo2) && is_line(geo1)) {
            AddPointOnLineCons(id, geo2_idx, geo1_idx);
        }
        break;
    case ConsType::PointOnPerpBisector:
        if (is_point(geo1) && is_line(geo2)) {
            AddPointOnPerpBisectorCons(id, geo1_idx, geo2_idx);
        } else if (is_point(geo2) && is_line(geo1)) {
            AddPointOnPerpBisectorCons(id, geo2_idx, geo1_idx);
        }
        break;
    case ConsType::MidpointOnLine:
        if (is_line(geo1) && is_line(geo2)) {
            AddMidpointOnLineCons(id, geo1_idx, geo2_idx);
        }
        break;
    case ConsType::TangentCircumf:
        if (is_circle(geo1) && is_circle(geo2)) {
            AddTangentCircumfCons(id, geo1_idx, geo2_idx);
        }
        break;
        // derived
    case ConsType::Coincident:
        if (is_point(geo1) && is_point(geo2)) {
            AddP2PCoincidentCons(id, geo1_idx, PointPos::Mid, geo2_idx, PointPos::Mid);
        }
    case ConsType::Horizontal:
        if (is_point(geo1) && is_point(geo2)) {
            AddHorizontalCons(id, geo1_idx, PointPos::Mid, geo2_idx, PointPos::Mid);
        } else if (is_line(geo1) && is_none(geo2)) {
            AddHorizontalCons(id, geo1_idx, PointPos::Start, geo1_idx, PointPos::End);
        } else if (is_line(geo2) && is_none(geo1)) {
            AddHorizontalCons(id, geo2_idx, PointPos::Start, geo2_idx, PointPos::End);
        }
        break;
    case ConsType::Vertical:
        if (is_point(geo1) && is_point(geo2)) {
            AddVerticalCons(id, geo1_idx, PointPos::Mid, geo2_idx, PointPos::Mid);
        } else if (is_line(geo1) && is_none(geo2)) {
            AddVerticalCons(id, geo1_idx, PointPos::Start, geo1_idx, PointPos::End);
        } else if (is_line(geo2) && is_none(geo1)) {
            AddVerticalCons(id, geo2_idx, PointPos::Start, geo2_idx, PointPos::End);
        }
        break;
    case ConsType::PointOnCircle:
        if (is_point(geo1) && is_circle(geo2)) {
            AddPointOnCircleCons(id, geo1_idx, geo2_idx);
        } else if (is_point(geo2) && is_circle(geo1)) {
            AddPointOnCircleCons(id, geo2_idx, geo1_idx);
        }
        break;
    case ConsType::Tangent:
        if (is_line(geo1) && is_circle(geo2)) {
            AddL2CTangentCons(id, geo1_idx, geo2_idx);
        } else if (is_line(geo2) && is_circle(geo1)) {
            AddL2CTangentCons(id, geo2_idx, geo1_idx);
        } else if (is_circle(geo1) && is_circle(geo2)) {
            AddC2CTangentCons(id, geo1_idx, geo2_idx);
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

void Scene::AddP2PDistanceCons(ConsID id, int geo1, PointPos pos1, int geo2, PointPos pos2, double* value)
{
    assert(geo1 < m_geos.size() && geo2 < m_geos.size());
    auto p1 = m_geos[geo1].GetPointID(pos1);
    auto p2 = m_geos[geo2].GetPointID(pos2);
    assert(p1 < m_points.size() && p2 < m_points.size());

    m_gcs->addConstraintP2PDistance(m_points[p1], m_points[p2], value, id);

    ResetSolver();
}

void Scene::AddP2LDistanceCons(ConsID id, int point, int line, double* value)
{
    assert(point < m_geos.size() && line < m_geos.size());

    m_gcs->addConstraintP2LDistance(m_points[m_geos[point].index], m_lines[m_geos[line].index], value, id);

    ResetSolver();
}

void Scene::AddP2PAngleCons(ConsID id, int geo1, PointPos pos1, int geo2, PointPos pos2, double* value)
{
    assert(geo1 < m_geos.size() && geo2 < m_geos.size());
    auto p1 = m_geos[geo1].GetPointID(pos1);
    auto p2 = m_geos[geo2].GetPointID(pos2);
    assert(p1 < m_points.size() && p2 < m_points.size());

    m_gcs->addConstraintP2PAngle(m_points[p1], m_points[p2], value, id);

    ResetSolver();
}

void Scene::AddParallelCons(ConsID id, int line1, int line2)
{
    assert(line1 < m_geos.size() && line2 < m_geos.size());

    m_gcs->addConstraintParallel(m_lines[m_geos[line1].index], m_lines[m_geos[line2].index], id);

    ResetSolver();
}

void Scene::AddPerpendicularCons(ConsID id, int line1, int line2)
{
    assert(line1 < m_geos.size() && line2 < m_geos.size());

    m_gcs->addConstraintPerpendicular(m_lines[m_geos[line1].index], m_lines[m_geos[line2].index], id);

    ResetSolver();
}

void Scene::AddPointOnLineCons(ConsID id, int point, int line)
{
    assert(point < m_geos.size() && line < m_geos.size());

    m_gcs->addConstraintPointOnLine(m_points[m_geos[point].index], m_lines[m_geos[line].index], id);

    ResetSolver();
}

void Scene::AddPointOnPerpBisectorCons(ConsID id, int point, int line)
{
    assert(point < m_geos.size() && line < m_geos.size());

    m_gcs->addConstraintPointOnPerpBisector(m_points[m_geos[point].index], m_lines[m_geos[line].index], id);

    ResetSolver();
}

void Scene::AddMidpointOnLineCons(ConsID id, int line1, int line2)
{
    assert(line1 < m_geos.size() && line2 < m_geos.size());

    m_gcs->addConstraintMidpointOnLine(m_lines[m_geos[line1].index], m_lines[m_geos[line2].index], id);

    ResetSolver();
}

void Scene::AddTangentCircumfCons(ConsID id, int circle1, int circle2)
{
    assert(circle1 < m_geos.size() && circle2 < m_geos.size());

    m_gcs->addConstraintTangentCircumf(m_points[m_geos[circle1].mid_pt_idx], m_points[m_geos[circle2].mid_pt_idx], 
        m_circles[m_geos[circle1].index].rad, m_circles[m_geos[circle2].index].rad, false, id);

    ResetSolver();
}

void Scene::AddP2PCoincidentCons(ConsID id, int geo1, PointPos pos1, int geo2, PointPos pos2)
{
    assert(geo1 < m_geos.size() && geo2 < m_geos.size());
    auto p1 = m_geos[geo1].GetPointID(pos1);
    auto p2 = m_geos[geo2].GetPointID(pos2);
    assert(p1 < m_points.size() && p2 < m_points.size());

    m_gcs->addConstraintP2PCoincident(m_points[p1], m_points[p2], id);

    ResetSolver();
}

void Scene::AddHorizontalCons(ConsID id, int geo1, PointPos pos1, int geo2, PointPos pos2)
{
    assert(geo1 < m_geos.size() && geo2 < m_geos.size());
    auto p1 = m_geos[geo1].GetPointID(pos1);
    auto p2 = m_geos[geo2].GetPointID(pos2);
    assert(p1 < m_points.size() && p2 < m_points.size());

    m_gcs->addConstraintHorizontal(m_points[p1], m_points[p2], id);

    ResetSolver();
}

void Scene::AddVerticalCons(ConsID id, int geo1, PointPos pos1, int geo2, PointPos pos2)
{
    assert(geo1 < m_geos.size() && geo2 < m_geos.size());
    auto p1 = m_geos[geo1].GetPointID(pos1);
    auto p2 = m_geos[geo2].GetPointID(pos2);
    assert(p1 < m_points.size() && p2 < m_points.size());

    m_gcs->addConstraintVertical(m_points[p1], m_points[p2], id);

    ResetSolver();
}

void Scene::AddPointOnCircleCons(ConsID id, int point, int circle)
{
    assert(point < m_geos.size() && circle < m_geos.size());

    m_gcs->addConstraintPointOnCircle(m_points[m_geos[point].index], m_circles[m_geos[circle].index], id);

    ResetSolver();
}

void Scene::AddL2CTangentCons(ConsID id, int line, int circle)
{
    assert(line < m_geos.size() && circle < m_geos.size());

    m_gcs->addConstraintTangent(m_lines[m_geos[line].index], m_circles[m_geos[circle].index], id);

    ResetSolver();
}

void Scene::AddC2CTangentCons(ConsID id, int circle1, int circle2)
{
    assert(circle1 < m_geos.size() && circle2 < m_geos.size());

    m_gcs->addConstraintTangent(m_circles[m_geos[circle1].index], m_circles[m_geos[circle2].index], id);

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
    }
}

}