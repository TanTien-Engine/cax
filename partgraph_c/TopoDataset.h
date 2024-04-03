#pragma once

#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shell.hxx>

namespace partgraph
{

class TopoShape
{
public:
	TopoShape() {}
	TopoShape(const TopoDS_Shape& shape)
		: m_shape(shape)
	{
	}

	auto& GetShape() const { return m_shape; }

	void SetTime(int time) { m_time = time; }
	int GetTime() const { return m_time; }

private:
	TopoDS_Shape m_shape;

	int m_time = -1;

}; // TopoShape

class TopoEdge
{
public:
	TopoEdge() {}
	TopoEdge(const TopoDS_Edge& edge)
		: m_edge(edge)
	{
	}

	auto& GetEdge() const { return m_edge; }

private:
	TopoDS_Edge m_edge;

}; // TopoEdge

class TopoWire
{
public:
	TopoWire() {}
	TopoWire(const TopoDS_Wire& wire)
		: m_wire(wire)
	{
	}

	auto& GetWire() const { return m_wire; }

private:
	TopoDS_Wire m_wire;

}; // TopoWire

class TopoFace
{
public:
	TopoFace() {}
	TopoFace(const TopoDS_Face& face)
		: m_face(face)
	{
	}

	auto& GetFace() const { return m_face; }

	void SetInactive(bool inactive) { m_inactive = inactive; }
	bool GetInactive() const { return m_inactive; }

private:
	TopoDS_Face m_face;

	bool m_inactive = false;

}; // TopoFace

class TopoShell
{
public:
	TopoShell() {}
	TopoShell(const TopoDS_Shell& shell)
		: m_shell(shell)
	{
	}

	auto& GetShell() const { return m_shell; }

private:
	TopoDS_Shell m_shell;

}; // TopoShell

}