#include "brepir_c/File.h"
#include "brepir_c/Data.h"

#include <fstream>
#include <iostream>

namespace
{

struct FileHeader 
{
    uint32_t magic = 0x43414449; // "CADI" (CAD IR)
    uint32_t version = 1;
    uint32_t header_count;
    uint32_t data_count;
};

}

namespace brepir
{

bool File::Save(const std::string& filename, const GeometryPool& pool)
{
    std::ofstream os(filename, std::ios::binary);
    if (!os) return false;

    FileHeader fHead;
    fHead.header_count = static_cast<uint32_t>(pool.headers.size());
    fHead.data_count   = static_cast<uint32_t>(pool.data_pool.size());
    os.write(reinterpret_cast<const char*>(&fHead), sizeof(FileHeader));

    if (!pool.headers.empty()) {
        os.write(reinterpret_cast<const char*>(pool.headers.data()), 
                    pool.headers.size() * sizeof(Header));
    }

    if (!pool.data_pool.empty()) {
        os.write(reinterpret_cast<const char*>(pool.data_pool.data()), 
                    pool.data_pool.size() * sizeof(double));
    }

    os.close();
    return true;
}

bool File::Load(const std::string& filename, GeometryPool& pool)
{
    std::ifstream is(filename, std::ios::binary);
    if (!is) return false;

    FileHeader fHead;
    is.read(reinterpret_cast<char*>(&fHead), sizeof(FileHeader));
    if (fHead.magic != 0x43414449) return false;

    pool.headers.resize(fHead.header_count);
    if (fHead.header_count > 0) {
        is.read(reinterpret_cast<char*>(pool.headers.data()),
            fHead.header_count * sizeof(Header));
    }

    pool.data_pool.resize(fHead.data_count);
    if (fHead.data_count > 0) {
        is.read(reinterpret_cast<char*>(pool.data_pool.data()),
            fHead.data_count * sizeof(double));
    }

    is.close();
    return true;
}

}