// Append this block to brepdb.ves.inc inside the QUOTE(...) section,
// after the existing BrepDB class declaration.

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
