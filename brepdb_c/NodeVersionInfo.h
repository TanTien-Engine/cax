#pragma once

#include <objcomp/Component.h>

#include <cstdint>
#include <string>

namespace brepdb
{

// Component attached to each graph::Node representing a VersionNode.
// Carries the version metadata needed by the editor for display and interaction.
class NodeVersionInfo : public objcomp::Component
{
public:
    NodeVersionInfo(uint32_t version_id, const std::string& desc,
                    uint32_t op_type, uint64_t timestamp, bool is_current)
        : m_version_id(version_id)
        , m_desc(desc)
        , m_op_type(op_type)
        , m_timestamp(timestamp)
        , m_is_current(is_current)
    {
    }

    virtual const char* Type() const override { return "version_info"; }
    virtual objcomp::CompID TypeID() const override { return objcomp::GetCompTypeID<NodeVersionInfo>(); }
    virtual NodeVersionInfo* Clone() const override { return nullptr; }

    uint32_t           GetVersionId() const { return m_version_id; }
    const std::string& GetDesc()      const { return m_desc; }
    uint32_t           GetOpType()    const { return m_op_type; }
    uint64_t           GetTimestamp()  const { return m_timestamp; }
    bool               IsCurrent()    const { return m_is_current; }

    void SetCurrent(bool c) { m_is_current = c; }

private:
    uint32_t    m_version_id;
    std::string m_desc;
    uint32_t    m_op_type;
    uint64_t    m_timestamp;
    bool        m_is_current;

}; // NodeVersionInfo

} // namespace brepdb
