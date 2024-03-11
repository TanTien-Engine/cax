#pragma once

#include "modules/script/Proxy.h"

#include <vessel.h>

#include <memory>

namespace partgraph
{

class TopoShape;
class TopoEdge;
class TopoWire;
class TopoFace;
class TopoShell;

template<typename T>
void return_topo_obj(const std::shared_ptr<T>& obj, const char* class_name)
{
    if (!obj) {
        ves_set_nil(0);
        return;
    }

    ves_pop(ves_argnum());

    ves_pushnil();
    ves_import_class("partgraph", class_name);
    auto proxy = (tt::Proxy<T>*)ves_set_newforeign(0, 1, sizeof(tt::Proxy<T>));
    proxy->obj = obj;
    ves_pop(1);
}

void return_topo_shape(const std::shared_ptr<TopoShape>& shape);
void return_topo_edge(const std::shared_ptr<TopoEdge>& edge);
void return_topo_wire(const std::shared_ptr<TopoWire>& wire);
void return_topo_face(const std::shared_ptr<TopoFace>& face);
void return_topo_shell(const std::shared_ptr<TopoShell>& shell);

}