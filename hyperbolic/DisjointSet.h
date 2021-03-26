#pragma once
#ifndef _DISJOINT_SETS_H_
#define _DISJOINT_SETS_H_

#include <vector>
#include <unordered_map>

template <class V>
class DisjointSet
{
public:
	//
	inline void insert(V v)
	{
		if (m_ranks.find(v) != m_ranks.end()) return;
		m_ranks[v] = 0;
		m_parents[v] = v;
		++m_count;
	}

	//
	inline V find(V v)
	{
		while (v != m_parents[v])
		{
			V u = m_parents[v];
			u = m_parents[v] = m_parents[u]; // compress path
			v = u;
		}
		return v;
	}

	//
	inline void join(V v, V u)
	{
		V rv = find(v);
		V ru = find(u);
		if (rv == ru) return;

		int dv = m_ranks[rv];
		int du = m_ranks[ru];

		if (rv < ru)
			m_parents[rv] = ru;
		else if (rv > ru)
			m_parents[ru] = rv;
		else
		{
			m_parents[ru] = rv;
			++m_ranks[rv];
		}

		--m_count;
	}

	//
	inline int count() { return m_count; }

protected:
	//
	std::unordered_map<V, int> m_ranks;

	//
	std::unordered_map<V, V> m_parents;

	//
	int m_count;
};

#endif // !_DISJOINT_SETS_H_
