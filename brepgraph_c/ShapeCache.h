#pragma once

#include <cstdint>
#include <list>
#include <unordered_map>
#include <utility>
#include <variant>

namespace brepgraph
{

// LRU cache for computed values (primarily shapes).
// Keyed by IRNode NRef::id.  When capacity is exceeded the
// least-recently-used entry is evicted.
template<typename Val>
class LruCache
{
public:
	explicit LruCache(size_t capacity = 64) : m_capacity(capacity) {}

	Val* Get(uint32_t key)
	{
		auto it = m_index.find(key);
		if (it == m_index.end()) return nullptr;
		m_lru.splice(m_lru.begin(), m_lru, it->second);
		return &it->second->second;
	}

	const Val* Get(uint32_t key) const
	{
		auto it = m_index.find(key);
		if (it == m_index.end()) return nullptr;
		return &it->second->second;
	}

	void Put(uint32_t key, Val val)
	{
		auto it = m_index.find(key);
		if (it != m_index.end())
		{
			it->second->second = std::move(val);
			m_lru.splice(m_lru.begin(), m_lru, it->second);
			return;
		}
		Evict();
		m_lru.emplace_front(key, std::move(val));
		m_index[key] = m_lru.begin();
	}

	void Remove(uint32_t key)
	{
		auto it = m_index.find(key);
		if (it == m_index.end()) return;
		m_lru.erase(it->second);
		m_index.erase(it);
	}

	void Clear()
	{
		m_lru.clear();
		m_index.clear();
	}

	size_t Size()     const { return m_lru.size(); }
	size_t Capacity() const { return m_capacity; }

	void SetCapacity(size_t cap)
	{
		m_capacity = cap;
		while (m_lru.size() > m_capacity)
		{
			m_index.erase(m_lru.back().first);
			m_lru.pop_back();
		}
	}

private:
	void Evict()
	{
		while (m_lru.size() >= m_capacity)
		{
			m_index.erase(m_lru.back().first);
			m_lru.pop_back();
		}
	}

	size_t m_capacity;
	std::list<std::pair<uint32_t, Val>> m_lru;
	std::unordered_map<uint32_t,
		typename std::list<std::pair<uint32_t, Val>>::iterator> m_index;
};

} // namespace brepgraph
