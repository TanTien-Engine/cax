#include "Scene.h"

#include <geoshape/Point2D.h>
#include <geoshape/Line2D.h>

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
    case ConstraintType::Distance:
    {
        if (geo1 >= 0 && geo2 >= 0 &&
            cons.geo1->GetType() == gs::ShapeType2D::Point &&
            cons.geo2->GetType() == gs::ShapeType2D::Point) 
        {
            AddDistanceCons(geo1, PointPos::Mid, geo2, PointPos::Mid, &cons.value);
        } 
        else if (geo1 >= 0 && cons.geo1->GetType() == gs::ShapeType2D::Line) 
        {
            AddDistanceCons(geo1, &cons.value);
        }
    }
        break;
    case ConstraintType::Horizontal:
        if (geo1 >= 0 && geo2 >= 0 &&
            cons.geo1->GetType() == gs::ShapeType2D::Point &&
            cons.geo2->GetType() == gs::ShapeType2D::Point) 
        {
            AddHorizontalCons(geo1, PointPos::Mid, geo2, PointPos::Mid);
        } 
        else if (geo1 >= 0 && cons.geo1->GetType() == gs::ShapeType2D::Line) 
        {
            AddHorizontalCons(geo1);
        }
        break;
    case ConstraintType::Vertical:
        if (geo1 >= 0 && geo2 >= 0 &&
            cons.geo1->GetType() == gs::ShapeType2D::Point &&
            cons.geo2->GetType() == gs::ShapeType2D::Point) 
        {
            AddVerticalCons(geo1, PointPos::Mid, geo2, PointPos::Mid);
        } 
        else if (geo1 >= 0 && cons.geo1->GetType() == gs::ShapeType2D::Line) 
        {
            AddVerticalCons(geo1);
        }
        break;
    }
}
    
int Scene::Solve()
{
    BeforeSolve();
    int ret = m_gcs->solve();
    if (ret == GCS::Success) {
        m_gcs->applySolution();
        AfterSolve();
    }

    return ret;
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
    }
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

int Scene::AddDistanceCons(int point1, PointPos pos1, int point2, PointPos pos2, double* value)
{
    assert(point1 < m_geos.size() && point2 < m_geos.size());
    auto p1 = m_geos[point1].GetPointID(pos1);
    auto p2 = m_geos[point2].GetPointID(pos2);
    assert(p1 < m_points.size() && p2 < m_points.size());

    int tag = ++m_constraints_counter;
    m_gcs->addConstraintP2PDistance(m_points[p1], m_points[p2], value, tag);

    ResetSolver();

    return tag;
}

int Scene::AddDistanceCons(int line, double* value)
{
    return AddDistanceCons(line, PointPos::Start, line, PointPos::End, value);
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

int Scene::AddHorizontalCons(int line)
{
    assert(line < m_geos.size());
    assert(m_geos[line].shape->GetType() == gs::ShapeType2D::Line);
    return AddHorizontalCons(line, PointPos::Start, line, PointPos::End);
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

int Scene::AddVerticalCons(int line)
{
    assert(line < m_geos.size());
    assert(m_geos[line].shape->GetType() == gs::ShapeType2D::Line);
    return AddVerticalCons(line, PointPos::Start, line, PointPos::End);
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
    }
}

void Scene::AfterSolve()
{
    for (auto& geo : m_geos)
    {
        auto type = geo.shape->GetType();
        if (type == gs::ShapeType2D::Point)
        {
            auto dst = std::static_pointer_cast<gs::Point2D>(geo.shape);
            auto& src = m_points[geo.index];
            dst->SetPos(sm::vec2(
                static_cast<float>(*src.x), static_cast<float>(*src.y))
            );
        }
        else if (type == gs::ShapeType2D::Line)
        {
            auto dst = std::static_pointer_cast<gs::Line2D>(geo.shape);
            auto& src = m_lines[geo.index];
            dst->SetStart(sm::vec2(static_cast<float>(*src.p1.x), static_cast<float>(*src.p1.y)));
            dst->SetEnd(sm::vec2(static_cast<float>(*src.p2.x), static_cast<float>(*src.p2.y)));
        }
    }
}

}