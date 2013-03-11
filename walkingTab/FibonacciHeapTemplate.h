// ====================================================================
//      Original implementation of Fibonacci Heap is done by Max Winkler.
//	Then, its template version is implemented by Oktay Arslan.
// ====================================================================

#ifndef _FIBONACCI_HEAP_TEMPLATE_H_
#define _FIBONACCI_HEAP_TEMPLATE_H_

#include <cstdio>
#include <vector>
#include <iostream>
#include <vector>
#include <cmath>


const int MAXCOST = 100000;


#define MAX(X,Y) (X) > (Y) ? (X) : (Y)
#define MIN(X,Y) (X) < (Y) ? (X) : (Y)

enum HeapState
{
	HS_NULL, INSIDE_HEAP, OUTSIDE_HEAP
};

struct CostKey
{
    double key1;
    double key2;
    CostKey();

    friend bool operator==(const CostKey& x, const CostKey& y);
    friend bool operator<=(const CostKey& x, const CostKey& y);
    friend bool operator>=(const CostKey& x, const CostKey& y);
    friend bool operator<(const CostKey& x, const CostKey& y);
    friend bool operator>(const CostKey& x, const CostKey& y);
};


template <typename DataType, typename KeyType>
class HeapNode
{
public:
	HeapNode<DataType,KeyType> * parent;
	HeapNode<DataType,KeyType> * leftSibling, * rightSibling;
	HeapNode<DataType,KeyType> * children;
	HeapNode<DataType,KeyType> * pred;

	HeapNode();
	HeapNode(DataType data, KeyType key);


	DataType data;
	KeyType key;

	int rank;

	bool addChild(HeapNode * node);
	bool addSibling(HeapNode * node);
	bool remove();

	HeapNode* leftMostSibling();
	HeapNode* rightMostSibling();

private:
};

// =========================================================================
//	Implementation of class HeapNode
// =========================================================================

template <typename DataType, typename KeyType>
HeapNode<DataType,KeyType>::HeapNode(DataType data, KeyType key)
{
	this->key = key;
	this->data = data;
	parent = NULL;
	children = NULL;
	leftSibling = NULL;
	rightSibling = NULL;
	pred = NULL;
	rank = 0;

}



template <typename DataType, typename KeyType>
HeapNode<DataType,KeyType>::HeapNode()
{
	parent = NULL;
	children = NULL;
	leftSibling = NULL;
	rightSibling = NULL;
	pred = NULL;
	rank = 0;
}

template <typename DataType, typename KeyType>
bool HeapNode<DataType,KeyType>::addChild(HeapNode<DataType,KeyType> *node)
{
	if(children != NULL)
		children->addSibling(node);
	else
	{
		children = node;
		node->parent = this;
		rank = 1;
	}

	return true;
}

template <typename DataType, typename KeyType>
bool HeapNode<DataType,KeyType>::addSibling(HeapNode<DataType,KeyType> *node)
{
	HeapNode<DataType,KeyType>* temp = rightMostSibling();
	if(temp == NULL)
		return false;

	temp->rightSibling = node;
	node->leftSibling = temp;
	node->parent = this->parent;
	node->rightSibling = NULL;

	if(parent)
		parent->rank++;

	return true;
}
template <typename DataType, typename KeyType>
bool HeapNode<DataType,KeyType>::remove()
{

	if(parent)
	{
		parent->rank--;
		if(leftSibling)
			parent->children = leftSibling;
		else if(rightSibling)
			parent->children = rightSibling;
		else
			parent->children = NULL;
	}

	if(leftSibling)
		leftSibling->rightSibling = rightSibling;
	if(rightSibling)
		rightSibling->leftSibling = leftSibling;

	leftSibling = NULL;
	rightSibling = NULL;
	parent = NULL;
	// children

	return true;
}

template <typename DataType, typename KeyType>
HeapNode<DataType,KeyType>* HeapNode<DataType,KeyType>::leftMostSibling()
{
	if(this == NULL)
		return NULL;

	HeapNode<DataType,KeyType>* temp = this;
	while(temp->leftSibling){
		temp = temp->leftSibling;

	}
	return temp;
}

template <typename DataType, typename KeyType>
HeapNode<DataType,KeyType>* HeapNode<DataType,KeyType>::rightMostSibling()
{
	if(this == NULL)
		return NULL;

	HeapNode* temp = this;
	while(temp->rightSibling)
		temp = temp->rightSibling;
	return temp;
}



template<typename DataType, typename KeyType>
class FibonacciHeap
{
private:
	HeapNode<DataType, KeyType>* rootListByRank[100];

	bool link(HeapNode<DataType, KeyType>* root);
	HeapNode<DataType, KeyType>* minRoot;

public:

	FibonacciHeap();
	FibonacciHeap(HeapNode<DataType, KeyType> *root);

	~FibonacciHeap();

	bool isEmpty();
	bool insertVertex(HeapNode<DataType, KeyType> * node);
	void decreaseKey(KeyType delta, HeapNode<DataType, KeyType>* vertex);

	bool isContain(HeapNode<DataType, KeyType> * node);

	HeapNode<DataType, KeyType>* findMin();
	HeapNode<DataType, KeyType>* deleteMin();
};





// =========================================================================
//	Implementation of class Fibonacci Heap
// =========================================================================

template <typename DataType, typename KeyType>
FibonacciHeap<DataType,KeyType>::FibonacciHeap()
{
	minRoot = NULL;
}

template <typename DataType, typename KeyType>
FibonacciHeap<DataType,KeyType>::FibonacciHeap(HeapNode<DataType,KeyType> *root)
{
	this->minRoot = root;
	minRoot->parent = NULL;
	minRoot->children = NULL;
	minRoot->leftSibling = NULL;
	minRoot->rightSibling = NULL;
	minRoot->rank = 0;
}

template <typename DataType, typename KeyType>
FibonacciHeap<DataType,KeyType>::~FibonacciHeap()
{
	delete[] rootListByRank;
}

template <typename DataType, typename KeyType>
bool FibonacciHeap<DataType,KeyType>::isEmpty()
{
	return (minRoot == NULL);
}

template <typename DataType, typename KeyType>
bool FibonacciHeap<DataType,KeyType>::insertVertex(HeapNode<DataType,KeyType> * node)
{
	if(node == NULL)
		return false;

	if(minRoot == NULL)
		minRoot = node;
	else
	{
		minRoot->addSibling(node);
		if(minRoot->key > node->key)
			minRoot = node;
	}
	return true;
}

template <typename DataType, typename KeyType>
HeapNode<DataType,KeyType>* FibonacciHeap<DataType,KeyType>::findMin()
{
	return minRoot;
}

template <typename DataType, typename KeyType>
HeapNode<DataType,KeyType>* FibonacciHeap<DataType, KeyType>::deleteMin()
{
	HeapNode<DataType,KeyType> *temp = minRoot->children->leftMostSibling();
	HeapNode<DataType,KeyType> *nextTemp = NULL;

	// Adding Children to root list
	while(temp != NULL)
	{
		nextTemp = temp->rightSibling; // Save next Sibling
		temp->remove();
		minRoot->addSibling(temp);
		temp = nextTemp;
	}

	// Select the left-most sibling of minRoot
	temp = minRoot->leftMostSibling();

	// Remove minRoot and set it to any sibling, if there exists one
	if(temp == minRoot)
	{
		if(minRoot->rightSibling)
			temp = minRoot->rightSibling;
		else
		{
			// Heap is obviously empty
			HeapNode<DataType,KeyType>* out = minRoot;
			minRoot->remove();
			minRoot = NULL;
			return out;
		}
	}
	HeapNode<DataType,KeyType>* out = minRoot;
	minRoot->remove();
	minRoot = temp;

	// Initialize list of roots
	for(int i=0; i<100; i++)
		rootListByRank[i] = NULL;

	while(temp)
	{
		// Check if key of current vertex is smaller than the key of minRoot
		if(temp->key < minRoot->key)
		{
			minRoot = temp;
		}

		nextTemp = temp->rightSibling;
		link(temp);
		temp = nextTemp;
	}

	return out;
}

template <typename DataType, typename KeyType>
bool FibonacciHeap<DataType,KeyType>::link(HeapNode<DataType,KeyType>* root)
{
	// Insert Vertex into root list
	if(rootListByRank[root->rank] == NULL)
	{
		rootListByRank[root->rank] = root;
		return false;
	}
	else
	{
		// Link the two roots
		HeapNode<DataType,KeyType>* linkVertex = rootListByRank[root->rank];
		rootListByRank[root->rank] = NULL;

		if(root->key < linkVertex->key || root == minRoot)
		{
			linkVertex->remove();
			root->addChild(linkVertex);
			if(rootListByRank[root->rank] != NULL)
				link(root);
			else
				rootListByRank[root->rank] = root;
		}
		else
		{
			root->remove();
			linkVertex->addChild(root);
			if(rootListByRank[linkVertex->rank] != NULL)
				link(linkVertex);
			else
				rootListByRank[linkVertex->rank] = linkVertex;
		}
		return true;
	}
}

template <typename DataType, typename KeyType>
void FibonacciHeap<DataType,KeyType>::decreaseKey(KeyType delta, HeapNode<DataType,KeyType>* vertex)
{
	vertex->key = delta;

	if(vertex->parent != NULL) // The vertex has a parent
	{
		// Remove vertex and add to root list
		vertex->remove();
		minRoot->addSibling(vertex);
	}
	// Check if key is smaller than the key of minRoot
	if(vertex->key < minRoot->key)
		minRoot = vertex;
}

template <typename DataType, typename KeyType>
bool  FibonacciHeap<DataType,KeyType>::isContain(HeapNode<DataType, KeyType> * node)
{
    if (node != minRoot && node->parent == NULL && node->leftSibling == NULL && node->rightSibling == NULL)
        return false;
    else
        return true;

}



#endif
