//! \file PriorityQueue.h
//! \brief Defines the <code>fullsail_ai::PriorityQueue</code> class template.
//! \author Cromwell D. Enage
#ifndef _FULLSAIL_AI_PATH_PLANNER_PRIORITY_QUEUE_H_
#define	_FULLSAIL_AI_PATH_PLANNER_PRIORITY_QUEUE_H_

#include <vector>
#include <deque>
#include <algorithm>


namespace fullsail_ai {

	//! \brief The open heap used by all cost-based search algorithms.
	//!
	//! This class template is basically a thin wrapper on top of both the <code>std::deque</code>
	//! class template and the <a href="http://www.sgi.com/tech/stl/">STL</a> heap operations.
	template <typename T, class Comp>
	class PriorityQueue
	{
		std::deque<T> open;

	public:
		//! \brief Constructs a new <code>%PriorityQueue</code> that heap-sorts nodes
		//! using the specified comparator.
		explicit PriorityQueue();

		//! \brief Returns <code>true</code> if the heap contains no nodes,
		//! <code>false</code> otherwise.
		bool empty() const;

		//! \brief Removes all nodes from the heap.
		//!
		//! \post
		//!   - <code>empty()</code>
		void clear();

		//! \brief Returns the number of nodes currently in the heap.
		std::size_t size() const;

		//! \brief Pushes the specified node onto the heap.
		//!
		//! The heap will maintain the ordering of its nodes during this operation.
		//!
		//! \post
		//!   - <code>! empty()</code>
		void push(T const& node);

		//! \brief Returns the least costly node in the heap.
		//!
		//! \pre
		//!   - <code>! empty()</code>
		T front() const;

		//! \brief Removes the least costly node from the heap.
		//!
		//! The heap will maintain the ordering of its nodes during this operation.
		//!
		//! \pre
		//!   - <code>! empty()</code>
		void pop();

		//! \brief Removes all instances of the specified node from the heap.
		void remove(T const& node);

		//! \brief Enumerates all nodes in the heap so far.
		//!
		//! \param   sorted  the container to which each node will be added.
		//!
		//! \pre
		//!   - There must be no <code>NULL</code> pointers in the heap.
		//! \post
		//!   - All nodes should be sorted by this heap's comparator.
		void enumerate(std::vector<T>& sorted) const;
	};

	template <typename T, class Comp>
	PriorityQueue<T, Comp>::PriorityQueue() : open()
	{
	}

	template <typename T, class Comp>
	bool PriorityQueue<T, Comp>::empty() const
	{
		return open.empty();
	}

	template <typename T, class Comp>
	void PriorityQueue<T, Comp>::clear()
	{
		open.clear();
	}

	template <typename T, class Comp>
	std::size_t PriorityQueue<T, Comp>::size() const
	{
		return open.size();
	}

	template <typename T, class Comp>
	void PriorityQueue<T, Comp>::push(T const& node)
	{
		open.insert(std::upper_bound(open.begin(), open.end(), node, Comp()), node);
	}

	template <typename T, class Comp>
	T PriorityQueue<T, Comp>::front() const
	{
		return open.back();
	}

	template <typename T, class Comp>
	void PriorityQueue<T, Comp>::pop()
	{
		open.pop_back();
	}

	template <typename T, class Comp>
	void PriorityQueue<T, Comp>::remove(T const& node)
	{
		open.erase(std::remove(open.begin(), open.end(), node), open.end());
	}

	template <typename T, class Comp>
	void PriorityQueue<T, Comp>::enumerate(std::vector<T>& sorted) const
	{
		sorted.resize(open.size());
		std::copy(open.begin(), open.end(), sorted.begin());
	}
}  // namespace fullsail_ai

#include <windows.h>
#include <tchar.h>
#define printf MessageBox(0,_T("printf is not permitted"), _T("PLEASE USE cout <<"), 0);
#define system MessageBox(0,_T("system is not permitted"), _T("YIPE!"), 0);

#endif  // _FULLSAIL_AI_PATH_PLANNER_PRIORITY_QUEUE_H_

