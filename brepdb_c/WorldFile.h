#pragma once

#include <string>

namespace brepdb
{

class BRepWorld;

class WorldFile
{
public:
    static bool Save(const std::string& filename, const BRepWorld& world);
    static bool Load(const std::string& filename, BRepWorld& world);

}; // WorldFile

}
