#define QUOTE(...) #__VA_ARGS__
static const char* brepdbModuleSource = QUOTE(

foreign class BrepWorld
{
	init() {}

	foreign serialize(shape)
	foreign deserialize()
	foreign save(filepath)
	foreign load(filepath)
	foreign entity_count()
}

foreign class BrepDB
{
	init() {}
	init(db) {}

	foreign build(shape)
	foreign query(aabb)
}

foreign class VersionTree
{
	init() {}

	foreign init_pool(ir, desc)
	foreign commit(ir, desc)

	foreign undo()
	foreign redo()
	foreign checkout(node_id)

	// Query.
	foreign get_current_id()
	foreign get_node_count()
	foreign can_undo()
	foreign can_redo()
	foreign get_node_desc(node_id)
	foreign get_children(node_id)

	// Persistence.
	foreign save(filepath)
	foreign load(filepath)
	foreign clear()
}

foreign class VersionGraph
{
	init(vt) {}

	foreign get_graph()
}

);