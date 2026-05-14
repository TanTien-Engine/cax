#include "TopoNaming.h"
#include "HistGraph.h"

#include <partgraph_c/BRepHistory.h>
#include <partgraph_c/TopoShape.h>

#include <TopAbs_ShapeEnum.hxx>

#include <cstring>

namespace
{

void Merge(breptopo::TopoNaming::PidMap& dst, const breptopo::HistGraph::PartialPidMap& src)
{
	for (const auto& kv : src) {
		// Each old uid is unique across all 4 types (type bits in the high bits
		// of CalcUID), so simple insertion is safe.
		dst.emplace(kv.first, kv.second);
	}
}

}

namespace breptopo
{

TopoNaming::TopoNaming()
{
	m_vertex_hg = std::make_shared<HistGraph>();
	m_edge_hg = std::make_shared<HistGraph>();
	m_face_hg = std::make_shared<HistGraph>();
	m_solid_hg = std::make_shared<HistGraph>();
}

TopoNaming::PidMap TopoNaming::Update(const partgraph::TopoShape& new_shape, uint32_t op_id)
{
	PidMap pm;
	Merge(pm, m_vertex_hg->Update(partgraph::BRepHistory(new_shape), TopAbs_VERTEX, op_id));
	Merge(pm, m_edge_hg->Update(partgraph::BRepHistory(new_shape), TopAbs_EDGE, op_id));
	Merge(pm, m_face_hg->Update(partgraph::BRepHistory(new_shape), TopAbs_FACE, op_id));
	Merge(pm, m_solid_hg->Update(partgraph::BRepHistory(new_shape), TopAbs_SOLID, op_id));
	return pm;
}

TopoNaming::PidMap TopoNaming::Update(BRepBuilderAPI_MakeShape& builder, const partgraph::TopoShape& new_shape,
	                                  const partgraph::TopoShape& old_shape, uint32_t op_id)
{
	PidMap pm;
	Merge(pm, m_vertex_hg->Update(partgraph::BRepHistory(builder, TopAbs_VERTEX, new_shape, old_shape), TopAbs_VERTEX, op_id));
	Merge(pm, m_edge_hg->Update(partgraph::BRepHistory(builder, TopAbs_EDGE, new_shape, old_shape), TopAbs_EDGE, op_id));
	Merge(pm, m_face_hg->Update(partgraph::BRepHistory(builder, TopAbs_FACE, new_shape, old_shape), TopAbs_FACE, op_id));
	Merge(pm, m_solid_hg->Update(partgraph::BRepHistory(builder, TopAbs_SOLID, new_shape, old_shape), TopAbs_SOLID, op_id));
	return pm;
}

TopoNaming::PidMap TopoNaming::Update(opencascade::handle<BRepTools_History> hist, const partgraph::TopoShape& new_shape,
	                                  const partgraph::TopoShape& old_shape, uint32_t op_id)
{
	PidMap pm;
	Merge(pm, m_vertex_hg->Update(partgraph::BRepHistory(hist, TopAbs_VERTEX, new_shape, old_shape), TopAbs_VERTEX, op_id));
	Merge(pm, m_edge_hg->Update(partgraph::BRepHistory(hist, TopAbs_EDGE, new_shape, old_shape), TopAbs_EDGE, op_id));
	Merge(pm, m_face_hg->Update(partgraph::BRepHistory(hist, TopAbs_FACE, new_shape, old_shape), TopAbs_FACE, op_id));
	Merge(pm, m_solid_hg->Update(partgraph::BRepHistory(hist, TopAbs_SOLID, new_shape, old_shape), TopAbs_SOLID, op_id));
	return pm;
}

TopoNaming::PidMap TopoNaming::Update(const BRepOffset_MakeSimpleOffset& builder, const partgraph::TopoShape& old_shape, uint32_t op_id)
{
	PidMap pm;
	Merge(pm, m_vertex_hg->Update(partgraph::BRepHistory(builder, old_shape), TopAbs_VERTEX, op_id));
	Merge(pm, m_edge_hg->Update(partgraph::BRepHistory(builder, old_shape), TopAbs_EDGE, op_id));
	Merge(pm, m_face_hg->Update(partgraph::BRepHistory(builder, old_shape), TopAbs_FACE, op_id));
	Merge(pm, m_solid_hg->Update(partgraph::BRepHistory(builder, old_shape), TopAbs_SOLID, op_id));
	return pm;
}

void TopoNaming::MergeFrom(const TopoNaming& other)
{
	m_vertex_hg->MergeFrom(*other.m_vertex_hg);
	m_edge_hg->MergeFrom(*other.m_edge_hg);
	m_face_hg->MergeFrom(*other.m_face_hg);
	m_solid_hg->MergeFrom(*other.m_solid_hg);
	if (other.m_next_op > m_next_op)
		m_next_op = other.m_next_op;
}

void TopoNaming::BindShape(uint32_t uid, const TopoDS_Shape& shape)
{
	uint32_t type_id = (uid >> 29) & 0x07;
	switch (type_id) {
	case TopAbs_VERTEX: m_vertex_hg->BindShape(uid, shape); break;
	case TopAbs_EDGE:   m_edge_hg->BindShape(uid, shape);   break;
	case TopAbs_FACE:   m_face_hg->BindShape(uid, shape);   break;
	case TopAbs_SOLID:  m_solid_hg->BindShape(uid, shape);  break;
	default: break;
	}
}

uint32_t TopoNaming::NextOpId()
{
	return m_next_op++;
}

void TopoNaming::StoreToByteArray(uint8_t** buf, uint32_t& len) const
{
	uint8_t* vb = nullptr; uint32_t vl = 0;
	uint8_t* eb = nullptr; uint32_t el = 0;
	uint8_t* fb = nullptr; uint32_t fl = 0;
	uint8_t* sb = nullptr; uint32_t sl = 0;

	m_vertex_hg->StoreToByteArray(&vb, vl);
	m_edge_hg->StoreToByteArray(&eb, el);
	m_face_hg->StoreToByteArray(&fb, fl);
	m_solid_hg->StoreToByteArray(&sb, sl);

	// Layout: magic(4) + version(4) + m_next_op(4) + 4 sub-arrays (len+data each)
	uint32_t total = 4 + 4 + 4 + (4 + vl) + (4 + el) + (4 + fl) + (4 + sl);
	*buf = new uint8_t[total];
	uint8_t* p = *buf;
	len = total;

	auto write32 = [&](uint32_t v) { std::memcpy(p, &v, 4); p += 4; };

	std::memcpy(p, "TNMG", 4); p += 4;
	write32(1);
	write32(m_next_op);

	auto writeBlob = [&](uint8_t* data, uint32_t dlen) {
		write32(dlen);
		if (dlen > 0) { std::memcpy(p, data, dlen); p += dlen; }
	};

	writeBlob(vb, vl);
	writeBlob(eb, el);
	writeBlob(fb, fl);
	writeBlob(sb, sl);

	delete[] vb;
	delete[] eb;
	delete[] fb;
	delete[] sb;
}

bool TopoNaming::LoadFromByteArray(const uint8_t* buf, uint32_t len)
{
	if (len < 12) return false;

	const uint8_t* p = buf;

	auto read32 = [&]() -> uint32_t {
		uint32_t v; std::memcpy(&v, p, 4); p += 4; return v;
	};

	if (std::memcmp(p, "TNMG", 4) != 0) return false;
	p += 4;
	uint32_t version = read32();
	if (version != 1) return false;

	m_next_op = read32();

	auto readBlob = [&](HistGraph& hg) -> bool {
		uint32_t dlen = read32();
		if (dlen == 0) return true;
		bool ok = hg.LoadFromByteArray(p, dlen);
		p += dlen;
		return ok;
	};

	if (!readBlob(*m_vertex_hg)) return false;
	if (!readBlob(*m_edge_hg)) return false;
	if (!readBlob(*m_face_hg)) return false;
	if (!readBlob(*m_solid_hg)) return false;

	return true;
}

}
