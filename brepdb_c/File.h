#pragma once

#include <string>

namespace brepdb
{

struct GeometryPool;

class File 
{
public:
    static bool Save(const std::string& filename, const GeometryPool& pool);
    static bool Load(const std::string& filename, GeometryPool& pool);
    
}; // File

}