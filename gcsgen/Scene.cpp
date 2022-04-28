#include "Scene.h"

#include <geoshape/Point2D.h>
#include <geoshape/Line2D.h>
#include <geoshape/Circle.h>

namespace
{

bool is_null(const std::shared_ptr<gs::Shape2D>& shape) {
    return !shape;
}

bool is_point(const std::shared_ptr<gs::Shape2D>& shape) {
    return shape && shape->GetType() == gs::ShapeType2D::Point;
}

bool is_line(const std::shared_ptr<gs::Shape2D>& shape) {
    return shape && shape->GetType() == gs::ShapeType2D::Line;
}

bool is_circle(const std::shared_ptr<gs::Shape2D>& shape) {
    return shape && shape->GetType() == gs::ShapeType2D::Circle;
}

}

namespace gcsgen
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

void Scene::AddConstraint(const Constraint& cons)
{
    int geo1 = AddGeometry(cons.geo1);
    int geo2 = AddGeometry(cons.geo2);

    switch (cons.type)
    {
        // basic
    case ConstraintType::Distance:
        if (is_point(cons.geo1) && is_point(cons.geo2)) {
            AddP2PDistanceCons(geo1, PointPos::Mid, geo2, PointPos::Mid, &cons.value);
        } else if (is_point(cons.geo1) && is_line(cons.geo2)) {
            AddP2LDistanceCons(geo1, geo2, &cons.value);
        } else if (is_line(cons.geo1) && is_point(cons.geo2)) {
            AddP2LDistanceCons(geo2, geo1, &cons.value);
        } else if (is_line(cons.geo1) && is_null(cons.geo2)) {
            AddP2PDistanceCons(geo1, PointPos::Start, geo1, PointPos::End, &cons.value);
        } else if (is_line(cons.geo2) && is_null(cons.geo1)) {
            AddP2PDistanceCons(geo2, PointPos::Start, geo2, PointPos::End, &cons.value);
        } else if (is_line(cons.geo1) && is_line(cons.geo2)) {
            AddP2PDistanceCons(geo1, PointPos::Start, geo2, PointPos::Start, &cons.value);
        }
        break;
    case ConstraintType::Angle:
        if (is_point(cons.geo1) && is_point(cons.geo2)) {
            AddP2PAngleCons(geo1, PointPos::Mid, geo2, PointPos::Mid, &cons.value);
        } else if (is_line(cons.geo1) && is_null(cons.geo2)) {
            AddP2PAngleCons(geo1, PointPos::Start, geo1, PointPos::End, &cons.value);
        } else if (is_line(cons.geo2) && is_null(cons.geo1)) {
            AddP2PAngleCons(geo2, PointPos::Start, geo2, PointPos::End, &cons.value);
        }
        break;
    case ConstraintType::Parallel:
        if (is_line(cons.geo1) && is_line(cons.geo2)) {
            AddParallelCons(geo1, geo2);
        }
        break;
    case ConstraintType::Perpendicular:
        if (is_line(cons.geo1) && is_line(cons.geo2)) {
            AddPerpendicularCons(geo1, geo2);
        }
        break;
    case ConstraintType::PointOnLine:
        if (is_point(cons.geo1) && is_line(cons.geo2)) {
            AddPointOnLineCons(geo1, geo2);
        } else if (is_point(cons.geo2) && is_line(cons.geo1)) {
            AddPointOnLineCons(geo2, geo1);
        }
        break;
    case ConstraintType::PointOnPerpBisector:
        if (is_point(cons.geo1) && is_line(cons.geo2)) {
            AddPointOnPerpBisectorCons(geo1, geo2);
        } else if (is_point(cons.geo2) && is_line(cons.geo1)) {
            AddPointOnPerpBisectorCons(geo2, geo1);
        }
        break;
    case ConstraintType::MidpointOnLine:
        if (is_line(cons.geo1) && is_line(cons.geo2)) {
            AddMidpointOnLineCons(geo1, geo2);
        }
        break;
    case ConstraintType::TangentCircumf:
        if (is_circle(cons.geo1) && is_circle(cons.geo2)) {
            AddTangentCircumfCons(geo1, geo2);
        }
        break;
        // derived
    case ConstraintType::Coincident:
        if (is_point(cons.geo1) && is_point(cons.geo2)) {
            AddP2PCoincidentCons(geo1, PointPos::Mid, geo2, PointPos::Mid);
        }
    case ConstraintType::Horizontal:
        if (is_point(cons.geo1) && is_point(cons.geo2)) {
            AddHorizontalCons(geo1, PointPos::Mid, geo2, PointPos::Mid);
        } else if (is_line(cons.geo1) && is_null(cons.geo2)) {
            AddHorizontalCons(geo1, PointPos::Start, geo1, PointPos::End);
        } else if (is_line(cons.geo2) && is_null(cons.geo1)) {
            AddHorizontalCons(geo2, PointPos::Start, geo2, PointPos::End);
        }
        break;
    case ConstraintType::Vertical:
        if (is_point(cons.geo1) && is_point(cons.geo2)) {
            AddVerticalCons(geo1, PointPos::Mid, geo2, PointPos::Mid);
        } else if (is_line(cons.geo1) && is_null(cons.geo2)) {
            AddVerticalCons(geo1, PointPos::Start, geo1, PointPos::End);
        } else if (is_line(cons.geo2) && is_null(cons.geo1)) {
            AddVerticalCons(geo2, PointPos::Start, geo2, PointPos::End);
        }
        break;
    case ConstraintType::PointOnCircle:
        if (is_point(cons.geo1) && is_circle(cons.geo2)) {
            AddPointOnCircleCons(geo1, geo2);
        } else if (is_point(cons.geo2) && is_circle(cons.geo1)) {
            AddPointOnCircleCons(geo2, geo1);
        }
        break;
    case ConstraintType::Tangent:
        if (is_line(cons.geo1) && is_circle(cons.geo2)) {
            AddL2CTangentCons(geo1, geo2);
        } else if (is_line(cons.geo2) && is_circle(cons.geo1)) {
            AddL2CTangentCons(geo2, geo1);
        } else if (is_circle(cons.geo1) && is_circle(cons.geo2)) {
            AddC2CTangentCons(geo1, geo2);
        }
        break;
    }
}
    
bool Scene::Solve()
{
    bool dirty = false;

    BeforeSolve();
    int status = m_gcs->solve();
    if (status == GCS::Success) 
    {
        m_gcs->applySolution();
        if (AfterSolve()) {
            dirty = true;
        }
    }

    return dirty;
}

void Scene::Clear()
{
    for (auto& p : m_parameters) {
        delete p;
    }
    m_parameters.clear();

    m_points.clear();
    m_lines.clear();

    ClearConstraints();

    m_geos.clear();
}

void Scene::ClearConstraints()
{
    for (int i = 0; i < m_constraints_counter; ++i) {
        m_gcs->clearByTag(i + 1);
    }
    m_constraints_counter = 0;
}

void Scene::ResetSolver()
{
    m_gcs->declareUnknowns(m_parameters);
    m_gcs->initSolution(m_default_solver_redundant);
}

int Scene::AddGeometry(const std::shared_ptr<gs::Shape2D>& geo)
{
    if (!geo) {
        return -1;
    }

    auto itr = m_geo_id.find(geo);
    if (itr != m_geo_id.end()) {
        return itr->second;
    }

    int ret = -1;
    switch (geo->GetType())
    {
    case gs::ShapeType2D::Point:
        ret = AddPoint(std::static_pointer_cast<gs::Point2D>(geo));
        break;
    case gs::ShapeType2D::Line:
        ret = AddLine(std::static_pointer_cast<gs::Line2D>(geo));
        break;
    case gs::ShapeType2D::Circle:
        ret = AddCircle(std::static_pointer_cast<gs::Circle>(geo));
        break;
    }

    m_geo_id[geo] = ret;

    return ret;
}

int Scene::AddPoint(const std::shared_ptr<gs::Point2D>& pt)
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
    geo.shape = pt;
    geo.index = point_id;
    geo.start_pt_idx = point_id;
    geo.mid_pt_idx = point_id;
    geo.end_pt_idx = point_id;

    int geo_id = static_cast<int>(m_geos.size());
    m_geos.push_back(geo);
    return geo_id;
}

int Scene::AddLine(const std::shared_ptr<gs::Line2D>& line)
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
    geo.shape = line;
    geo.start_pt_idx = static_cast<int>(m_points.size());
    geo.end_pt_idx   = geo.start_pt_idx + 1;
    m_points.push_back(p1);
    m_points.push_back(p2);

    GCS::Line l;
    l.p1 = p1;
    l.p2 = p2;
    geo.index = m_lines.size();
    m_lines.push_back(l);

    int geo_id = static_cast<int>(m_geos.size());
    m_geos.push_back(geo);
    return geo_id;
}

int Scene::AddCircle(const std::shared_ptr<gs::Circle>& circle)
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
    geo.shape = circle;
    geo.mid_pt_idx = static_cast<int>(m_points.size());
    m_points.push_back(center);
        
    GCS::Circle c;
    c.center = center;
    c.rad    = r;
    geo.index = m_circles.size();
    m_circles.push_back(c);

    int geo_id = static_cast<int>(m_geos.size());
    m_geos.push_back(geo);
    return geo_id;
}

int Scene::AddP2PDistanceCons(int geo1, PointPos pos1, int geo2, PointPos pos2, double* value)
{
    assert(geo1 < m_geos.size() && geo2 < m_geos.size());
    auto p1 = m_geos[geo1].GetPointID(pos1);
    auto p2 = m_geos[geo2].GetPointID(pos2);
    assert(p1 < m_points.size() && p2 < m_points.size());

    int tag = ++m_constraints_counter;
    m_gcs->addConstraintP2PDistance(m_points[p1], m_points[p2], value, tag);

    ResetSolver();

    return tag;
}

int Scene::AddP2LDistanceCons(int point, int line, double* value)
{
    assert(point < m_geos.size() && line < m_geos.size());
    assert(m_geos[point].shape->GetType() == gs::ShapeType2D::Point
        && m_geos[line].shape->GetType() == gs::ShapeType2D::Line);

    int tag = ++m_constraints_counter;
    m_gcs->addConstraintP2LDistance(m_points[m_geos[point].index], m_lines[m_geos[line].index], value, tag);

    ResetSolver();

    return tag;
}

int Scene::AddP2PAngleCons(int geo1, PointPos pos1, int geo2, PointPos pos2, double* value)
{
    assert(geo1 < m_geos.size() && geo2 < m_geos.size());
    auto p1 = m_geos[geo1].GetPointID(pos1);
    auto p2 = m_geos[geo2].GetPointID(pos2);
    assert(p1 < m_points.size() && p2 < m_points.size());

    int tag = ++m_constraints_counter;
    m_gcs->addConstraintP2PAngle(m_points[p1], m_points[p2], value, tag);

    ResetSolver();

    return tag;
}

int Scene::AddParallelCons(int line1, int line2)
{
    assert(line1 < m_geos.size() && line2 < m_geos.size());
    assert(m_geos[line1].shape->GetType() == gs::ShapeType2D::Line
        && m_geos[line2].shape->GetType() == gs::ShapeType2D::Line);

    int tag = ++m_constraints_counter;
    m_gcs->addConstraintParallel(m_lines[m_geos[line1].index], m_lines[m_geos[line2].index], tag);

    ResetSolver();

    return tag;
}

int Scene::AddPerpendicularCons(int line1, int line2)
{
    assert(line1 < m_geos.size() && line2 < m_geos.size());
    assert(m_geos[line1].shape->GetType() == gs::ShapeType2D::Line
        && m_geos[line2].shape->GetType() == gs::ShapeType2D::Line);

    int tag = ++m_constraints_counter;
    m_gcs->addConstraintPerpendicular(m_lines[m_geos[line1].index], m_lines[m_geos[line2].index], tag);

    ResetSolver();

    return tag;
}

int Scene::AddPointOnLineCons(int point, int line)
{
    assert(point < m_geos.size() && line < m_geos.size());
    assert(m_geos[point].shape->GetType() == gs::ShapeType2D::Point
        && m_geos[line].shape->GetType() == gs::ShapeType2D::Line);

    int tag = ++m_constraints_counter;
    m_gcs->addConstraintPointOnLine(m_points[m_geos[point].index], m_lines[m_geos[line].index], tag);

    ResetSolver();

    return tag;
}

int Scene::AddPointOnPerpBisectorCons(int point, int line)
{
    assert(point < m_geos.size() && line < m_geos.size());
    assert(m_geos[point].shape->GetType() == gs::ShapeType2D::Point
        && m_geos[line].shape->GetType() == gs::ShapeType2D::Line);

    int tag = ++m_constraints_counter;
    m_gcs->addConstraintPointOnPerpBisector(m_points[m_geos[point].index], m_lines[m_geos[line].index], tag);

    ResetSolver();

    return tag;
}

int Scene::AddMidpointOnLineCons(int line1, int line2)
{
    assert(line1 < m_geos.size() && line2 < m_geos.size());
    assert(m_geos[line1].shape->GetType() == gs::ShapeType2D::Line
        && m_geos[line2].shape->GetType() == gs::ShapeType2D::Line);

    int tag = ++m_constraints_counter;
    m_gcs->addConstraintMidpointOnLine(m_lines[m_geos[line1].index], m_lines[m_geos[line2].index], tag);

    ResetSolver();

    return tag;
}

int Scene::AddTangentCircumfCons(int circle1, int circle2)
{
    assert(circle1 < m_geos.size() && circle2 < m_geos.size());
    assert(m_geos[circle1].shape->GetType() == gs::ShapeType2D::Circle
        && m_geos[circle2].shape->GetType() == gs::ShapeType2D::Circle);

    int tag = ++m_constraints_counter;
    m_gcs->addConstraintTangentCircumf(m_points[m_geos[circle1].mid_pt_idx], m_points[m_geos[circle2].mid_pt_idx], 
        m_circles[m_geos[circle1].index].rad, m_circles[m_geos[circle2].index].rad, false, tag);

    ResetSolver();

    return tag;
}

int Scene::AddP2PCoincidentCons(int geo1, PointPos pos1, int geo2, PointPos pos2)
{
    assert(geo1 < m_geos.size() && geo2 < m_geos.size());
    auto p1 = m_geos[geo1].GetPointID(pos1);
    auto p2 = m_geos[geo2].GetPointID(pos2);
    assert(p1 < m_points.size() && p2 < m_points.size());

    int tag = ++m_constraints_counter;
    m_gcs->addConstraintP2PCoincident(m_points[p1], m_points[p2], tag);

    ResetSolver();

    return tag;
}

int Scene::AddHorizontalCons(int geo1, PointPos pos1, int geo2, PointPos pos2)
{
    assert(geo1 < m_geos.size() && geo2 < m_geos.size());
    auto p1 = m_geos[geo1].GetPointID(pos1);
    auto p2 = m_geos[geo2].GetPointID(pos2);
    assert(p1 < m_points.size() && p2 < m_points.size());

    int tag = ++m_constraints_counter;
    m_gcs->addConstraintHorizontal(m_points[p1], m_points[p2], tag);

    ResetSolver();

    return tag;
}

int Scene::AddVerticalCons(int geo1, PointPos pos1, int geo2, PointPos pos2)
{
    assert(geo1 < m_geos.size() && geo2 < m_geos.size());
    auto p1 = m_geos[geo1].GetPointID(pos1);
    auto p2 = m_geos[geo2].GetPointID(pos2);
    assert(p1 < m_points.size() && p2 < m_points.size());

    int tag = ++m_constraints_counter;
    m_gcs->addConstraintVertical(m_points[p1], m_points[p2], tag);

    ResetSolver();

    return tag;
}

int Scene::AddPointOnCircleCons(int point, int circle)
{
    assert(point < m_geos.size() && circle < m_geos.size());
    assert(m_geos[point].shape->GetType() == gs::ShapeType2D::Point
        && m_geos[circle].shape->GetType() == gs::ShapeType2D::Circle);

    int tag = ++m_constraints_counter;
    m_gcs->addConstraintPointOnCircle(m_points[m_geos[point].index], m_circles[m_geos[circle].index], tag);

    ResetSolver();

    return tag;
}

int Scene::AddL2CTangentCons(int line, int circle)
{
    assert(line < m_geos.size() && circle < m_geos.size());
    assert(m_geos[line].shape->GetType() == gs::ShapeType2D::Line
        && m_geos[circle].shape->GetType() == gs::ShapeType2D::Circle);

    int tag = ++m_constraints_counter;
    m_gcs->addConstraintTangent(m_lines[m_geos[line].index], m_circles[m_geos[circle].index], tag);

    ResetSolver();

    return tag;
}

int Scene::AddC2CTangentCons(int circle1, int circle2)
{
    assert(circle1 < m_geos.size() && circle2 < m_geos.size());
    assert(m_geos[circle1].shape->GetType() == gs::ShapeType2D::Circle
        && m_geos[circle2].shape->GetType() == gs::ShapeType2D::Circle);

    int tag = ++m_constraints_counter;
    m_gcs->addConstraintTangent(m_circles[m_geos[circle1].index], m_circles[m_geos[circle2].index], tag);

    ResetSolver();

    return tag;
}

void Scene::BeforeSolve()
{
    for (auto& geo : m_geos)
    {
        auto type = geo.shape->GetType();
        if (type == gs::ShapeType2D::Point)
        {
            auto src = std::static_pointer_cast<gs::Point2D>(geo.shape);
            auto& dst = m_points[geo.index];
            *dst.x = static_cast<double>(src->GetPos().x);
            *dst.y = static_cast<double>(src->GetPos().y);
        }
        else if (type == gs::ShapeType2D::Line)
        {
            auto src = std::static_pointer_cast<gs::Line2D>(geo.shape);
            auto& dst = m_lines[geo.index];
            *dst.p1.x = static_cast<double>(src->GetStart().x);
            *dst.p1.y = static_cast<double>(src->GetStart().y);
            *dst.p2.x = static_cast<double>(src->GetEnd().x);
            *dst.p2.y = static_cast<double>(src->GetEnd().y);
        }
        else if (type == gs::ShapeType2D::Circle)
        {
            auto src = std::static_pointer_cast<gs::Circle>(geo.shape);
            auto& dst = m_circles[geo.index];
            *dst.center.x = static_cast<double>(src->GetCenter().x);
            *dst.center.y = static_cast<double>(src->GetCenter().y);
            *dst.rad = static_cast<double>(src->GetRadius());
        }
    }
}

bool Scene::AfterSolve()
{
    bool dirty = false;

    for (auto& geo : m_geos)
    {
        auto type = geo.shape->GetType();
        if (type == gs::ShapeType2D::Point)
        {
            auto& src = m_points[geo.index];
            auto dst = std::static_pointer_cast<gs::Point2D>(geo.shape);

            sm::vec2 pos(static_cast<float>(*src.x), static_cast<float>(*src.y));
            if (pos != dst->GetPos()) 
            {
                dst->SetPos(pos);
                dirty = true;
            }
        }
        else if (type == gs::ShapeType2D::Line)
        {
            auto& src = m_lines[geo.index];
            auto dst = std::static_pointer_cast<gs::Line2D>(geo.shape);

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
            auto& src = m_circles[geo.index];
            auto dst = std::static_pointer_cast<gs::Circle>(geo.shape);

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

    return dirty;
}

}