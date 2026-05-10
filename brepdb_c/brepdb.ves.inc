#define QUOTE(...) #__VA_ARGS__
static const char* brepdbModuleSource = QUOTE(

foreign class BrepIR
{
	init() {}

	foreign save(filepath)
	foreign load(filepath)

	foreign serialize(shape)
	foreign deserialize()
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

	// Initialize with a BrepIR pool as the root snapshot.
	foreign init_pool(ir, desc)

	// Commit a new BrepIR pool after a modeling op (fallback diff path).
	// Returns the new node_id.
	foreign commit(ir, desc)

	// Navigate; each returns a BrepIR or nil.
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