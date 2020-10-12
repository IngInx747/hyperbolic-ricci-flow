#pragma once
#ifndef _TREE_H_
#define _TREE_H_

#include <vector>
#include <memory>
#include <queue>

template <typename T>
class Tree
{
protected:
	struct TreeNode;
	using Node = std::shared_ptr<TreeNode>;

protected:
	struct TreeNode
	{
		TreeNode() {}
		TreeNode(const T& val) : value(val) {}

		std::vector<Node> children;
		TreeNode* parent = NULL;
		T value;
	};

public:
	class PathIterator
	{
	public:
		PathIterator() {}

		PathIterator(const Tree& tree) { reset(tree); }

		void clear()
		{
			m_queue = std::queue<Node>();
			m_count = 0;
		}

		void reset(const Tree& tree)
		{
			m_queue = std::queue<Node>();
			for (Node node : tree.m_root->children)
				m_queue.push(node);
			m_count = 0;
		}
		
		void operator++()
		{
			if (m_queue.empty()) return;
			Node node = m_queue.front();
			m_queue.pop();
			for (const Node& child : node->children) { m_queue.push(child); }
			++m_count;
		}

		void operator++(int)
		{
			if (m_queue.empty()) return;
			Node node = m_queue.front();
			m_queue.pop();
			for (const Node& child : node->children) { m_queue.push(child); }
			++m_count;
		}

		bool end() { return m_queue.empty(); }

		std::vector<T> operator*()
		{
			std::vector<T> path;
			if (m_queue.empty()) return path;
			TreeNode* ptrNode = m_queue.front().get();
			while (ptrNode && ptrNode->parent)
			{
				path.push_back(ptrNode->value);
				ptrNode = ptrNode->parent;
			}
			std::reverse(path.begin(), path.end());
			return path;
		}

		int count() { return m_count; }

	protected:
		std::queue<Node> m_queue;
		int m_count = 0;
	};

public:
	bool empty() { return !m_root; }

	void span_complete_tree(const std::vector<T>& generators, int level)
	{
		int n = (int)generators.size();
		m_root = std::make_shared<TreeNode>();
		m_level = level;

		std::queue<Node> q;
		q.push(m_root);

		for (int l = 0; l < level; ++l)
		{
			std::vector<Node> nodes; // nodes of next level

			while (!q.empty())
			{
				Node node = q.front();
				q.pop();
				
				T vp = node->value; // Mobius transform specification

				for (const T& v : generators)
				{
					if (v + vp == 0) continue; // Mobius transform specification

					Node child = std::make_shared<TreeNode>(v);
					node->children.push_back(child);
					child->parent = node.get();
					nodes.push_back(child);
				}
			}

			for (Node node : nodes)
				q.push(node);
		}
	}

	int level() { return m_level; }

protected:
	Node m_root;
	int m_level = 0;
};

#endif // !_TREE_H_
