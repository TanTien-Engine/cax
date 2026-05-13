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
	foreign export_step(filepath)
	foreign import_step(filepath)
}

foreign class BrepDB
{
	init(filename, overwrite) {}

	foreign build(shape)
	foreign query(aabb)
}

foreign class VersionTree
{
	init() {}

	foreign add_root(world, desc)
	foreign commit(root_id, world, desc)

	foreign undo(root_id)
	foreign redo(root_id)
	foreign checkout(root_id, node_id)

	// Query.
	foreign get_current_id(root_id)
	foreign get_node_count()
	foreign can_undo(root_id)
	foreign can_redo(root_id)
	foreign get_node_desc(node_id)
	foreign get_children(node_id)
	foreign find_root_of(node_id)

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