#include "OpNameVocabulary.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>

namespace deepbrep_data_gen
{

namespace {

deepbrep::FaceClass class_from_name(const std::string& s)
{
    if (s == "Stock")   return deepbrep::FaceClass::Stock;
    if (s == "Hole")    return deepbrep::FaceClass::Hole;
    if (s == "Slot")    return deepbrep::FaceClass::Slot;
    if (s == "Fillet")  return deepbrep::FaceClass::Fillet;
    if (s == "Chamfer") return deepbrep::FaceClass::Chamfer;
    if (s == "Pocket")  return deepbrep::FaceClass::Pocket;
    return deepbrep::FaceClass::Stock;
}

}  // anonymous

OpNameVocabulary::OpNameVocabulary()
{
    // IR-level op_names from comp_ops.cpp (used by AssignOpIds / HistGraph).
    m_table["cut"]     = deepbrep::FaceClass::Hole;
    m_table["fillet"]  = deepbrep::FaceClass::Fillet;
    m_table["chamfer"] = deepbrep::FaceClass::Chamfer;

    m_table["box"]       = deepbrep::FaceClass::Stock;
    m_table["fuse"]      = deepbrep::FaceClass::Stock;
    m_table["translate"] = deepbrep::FaceClass::Stock;
    m_table["offset"]    = deepbrep::FaceClass::Stock;
}

void OpNameVocabulary::Set(const std::string& op_name, deepbrep::FaceClass cls)
{
    m_table[op_name] = cls;
}

deepbrep::FaceClass OpNameVocabulary::Lookup(const std::string& op_name) const
{
    if (op_name.empty()) return deepbrep::FaceClass::Stock;
    auto it = m_table.find(op_name);
    return it == m_table.end() ? deepbrep::FaceClass::Stock : it->second;
}

bool OpNameVocabulary::LoadFromFile(const std::string& path)
{
    std::ifstream in(path);
    if (!in) return false;

    std::string line;
    while (std::getline(in, line)) {
        // Strip comments and trim.
        const auto hash = line.find('#');
        if (hash != std::string::npos) line.resize(hash);
        while (!line.empty() && std::isspace(static_cast<unsigned char>(line.back()))) line.pop_back();
        size_t begin = 0;
        while (begin < line.size() && std::isspace(static_cast<unsigned char>(line[begin]))) ++begin;
        if (begin >= line.size()) continue;

        const auto eq = line.find('=', begin);
        if (eq == std::string::npos) continue;
        std::string key   = line.substr(begin, eq - begin);
        std::string value = line.substr(eq + 1);

        // Trim around key/value (cheap inline pass).
        while (!key.empty()   && std::isspace(static_cast<unsigned char>(key.back())))   key.pop_back();
        while (!value.empty() && std::isspace(static_cast<unsigned char>(value.front()))) value.erase(value.begin());
        while (!value.empty() && std::isspace(static_cast<unsigned char>(value.back())))  value.pop_back();

        if (!key.empty()) m_table[key] = class_from_name(value);
    }
    return true;
}

}
