#pragma once

#include <math.h>
#include <memory.h>
#include <list>
#include <stack>
#include <QReadWriteLock>

//KD树节点
template <typename DATATYPE,
          int DIMENSION = 3>
class KDTreeNode
{
public:
    int m_nSplit;
    DATATYPE m_data[DIMENSION];
    void* m_extraData;
    KDTreeNode* m_pLeftTree;
    KDTreeNode* m_pRightTree;
    KDTreeNode* m_pParent;
    KDTreeNode() : m_pLeftTree(nullptr), m_pRightTree(nullptr), m_pParent(nullptr), m_extraData(nullptr) {}
    bool isLeaf() {return m_pLeftTree == nullptr && m_pRightTree == nullptr;}
    bool isRoot() {return m_pParent == nullptr;}
};

template <typename DATATYPE, int DIMENSION = 3>
DATATYPE distance (KDTreeNode<DATATYPE, DIMENSION>* l,
    KDTreeNode<DATATYPE, DIMENSION>* r)
{
    DATATYPE res = 0;
    if (l && r)
    {
        for (int i = 0 ; i < DIMENSION, ++i)
        {
            res += ((l->m_data[i] - r->m_data[i]) * (l->m_data[i] - r->m_data[i]));
        }
        res = sqrt(res);
    }
    return res;
}

template <typename DATATYPE, int DIMENSION = 3>
DATATYPE distance (KDTreeNode<DATATYPE, DIMENSION>* l,
   DATATYPE[DIMENSION] data)
{
    DATATYPE res = 0;
    if (l)
    {
        for (int i = 0 ; i < DIMENSION, ++i)
        {
            res += ((l->m_data[i] - data[i]) * (l->m_data[i] - data[i]));
        }
        res = sqrt(res);
    }
    return res;
}

template <typename DATATYPE, int DIMENSION = 3>
DATATYPE distance2HyperPlane (KDTreeNode<DATATYPE, DIMENSION>* l,
    DATATYPE[DIMENSION] data)
{
    double dRes = -1.0;
    if (l)
    {
        dRes = fabs(l->m_data[l->m_nSplit],
            data[l->m_nSplit]);
    }
    return dRes;
}

template <typename DATATYPE, int DIMENSION = 3>
bool sameNode(KDTreeNode<DATATYPE, DIMENSION>* l,
    KDTreeNode<DATATYPE, DIMENSION>* r,
    double dEpsilon)
{
    bool bRes = true;
    if (l && r)
    {
        for (int i = 0 ; i < DIMENSION; ++i)
        {
            if (fabs(l->m_data[i] - r->m_data[i]) > dEpsilon)
            {
                bRes = false;
                break;
            }
        }
    }
    return bRes;
}

template <typename DATATYPE, int DIMENSION = 3>
bool sameNode(KDTreeNode<DATATYPE, DIMENSION>* l,
    DATATYPE data[DIMENSION],
    double dEpsilon)
{
    bool bRes = true;
    if (l)
    {
        for (int i = 0 ; i < DIMENSION; ++i)
        {
            if (fabs(l->m_data[i] - data[i]) > dEpsilon)
            {
                bRes = false;
                break;
            }
        }
    }
    return bRes;
}

//KD树最近点
template <typename DATATYPE,
          int DIMENSION = 3>
class KDNearestNode
{
    KDTreeNode<DIMENSION, DATATYPE>* m_pNode;
    DATATYPE m_distance;
};

//KD树
template <typename DATATYPE,
          int DIMENSION = 3>
class KDTree
{
public:
    typedef typename KDNearestNode<DATATYPE, DIMENSION> nearestNodeType;
    typedef typename KDTreeNode<DATATYPE, DIMENSION> nodeType;
    typedef typename KDTree<DATATYPE, DIMENSION> tree;
    typedef typename std::list<nodeType*>::iterator splitIter;
public:
    KDTree(double dEpsilon, 
        int nRebuildThreshold = 50);
    ~KDTree();
private:
    nodeType* m_pRoot;
    int m_nModifiedCount;
    int m_nRebuildThreshold;
    int m_nNodeCount;
    double m_dEpsilon;
public:
    void rebuildTree(std::list<nodeType*>& vecDatas);
public:
    //增加一个节点
    nodeType* addNode(DATATYPE data[DIMENSION]);
    //删除一个节点
    void removeNode(DATATYPE data[DIMENSION]);
    //查找一个节点
    nodeType* findNode(DATATYPE data[DIMENSION]);
    //找距离最近的
    nearestNodeType findNearest(DATATYPE data[DIMENSION]);
    //找N个距离最近的
    std::vector<nearestNodeType> findNNearest(DATATYPE data[DIMENSION], int N) {}    //KNN，暂时不实现
    //清空树，释放内存
    void clearTree();
private:
    void freeNode(nodeType* pNode);
    std::list<nodeType*> traverseTree(nodeType* pNodeExcept = nullptr);
    void free(std::list<DATATYPE*>& datas);
    void innerRebuildTree(nodeType*& pNode, std::list<nodeType*>& vecDatas);
    void innerAddNode(nodeType* pParent, nodeType* pNode);
    nodeType* innerFindNode(nodeType* pNode, DATATYPE data[DIMENSION]);
    void innerTraverse(std::list<nodeType*>& vecDatas, nodeType* pNode, nodeType* pExceptNode);
private:
    splitIter splitNodesByVariance(std::list<nodeType*>& vecDatas, int& nDimension);
private:
    QReadWriteLock m_lock;
};

template <typename DATATYPE,
    int DIMENSION /*= 3*/>
    nearestNodeType KDTree<DATATYPE, DIMENSION>::findNearest(DATATYPE data[DIMENSION])
{
    m_lock.lockForRead();
    std::stack<nodeType*> oNodeStack;
    auto posTranverse [&] (nodeType* pNode) -> void
    {
        while (pNode)
        {
            oNodeStack.push(pNode);
            if (data[pNode->m_nSplit] < pNode->m_data[pNode->m_nSplit])
            {
                pNode = pNode->m_pLeftTree;
            }
            else
            {
                pNode = pNode->m_pRightTree;
            }
        }
    };

    nearestNodeType oRes;
    //参考wikipedia, 其实可以写成递归
    //1. 正向递归
    nodeType* pNode = m_pRoot;
    posTranverse(pNode);
    nodeType* pBest = oNodeStack.top();
    double dMinDis = distance(pBest, data);
    //2. 反向回溯
    while (!oNodeStack.empty())
    {
        pNode = oNodeStack.top();
        oNodeStack.pop();
        //判断是否需要进入另一边子空间
        if (distance2HyperPlane(pNode) < dMinDis)
        {
            if (data[pNode->m_nSplit] < pNode->m_data[pNode->m_nSplit])
            {
                posTranverse(pNode->m_pRightTree);
            }
            else
            {
                posTranverse(pNode->m_pLeftTree);
            }
        }
        double dDistance = distance(pNode, data);
        if (dDistance < dMinDis)
        {
            dMinDis = dDistance;
            pBest = pNode;
        }
    }
    oRes.m_pNode = pBest;
    oRes.m_distance = dMinDis;
    m_lock.unlock();
}

template <typename DATATYPE,
    int DIMENSION /*= 3*/>
    splitIter KDTree<DATATYPE, DIMENSION>::splitNodesByVariance(std::list<nodeType*>& vecDatas, int& nDimension)
{
    //求所有维度上的方差
    std::list<nodeType*> oRes;
    double dMaxVariance = 0.0;
    auto variance = [] (std::list<nodeType*>& oList, int nDimension) -> double
    {
        double dTotal = 0.0;
        for (auto iter = oList.begin(); iter != oList.end(); ++iter)
        {
            const nodeType* pNode = *iter;
            dTotal += pNode->m_data[nDimension];
        }
        dTotal /= (int)(oList.size());
        double dRes = 0.0;
        for (auto iter = oList.begin(); iter != oList.end(); ++iter)
        {
            const nodeType* pNode = *iter;
            double dDiff = (pNode->m_data[nDimension] - dTotal);
            dRes += dDiff * dDiff;
        }
        return dRes;
    };

    for (int i = 0 ; i < DIMENSION; ++i)
    {
        std::list<nodeType*> oDimList = vecDatas;
        std::sort(oDimList.begin(), oDimList.end(), [&] (nodeType* l, nodeType* r)->bool
        {
            return l->m_data[i] < r->m_data[i];
        });
        double dVariance = variance(oDimList);
        if (dVariance > dMaxVariance)
        {
            nDimension = i;
            dMaxVariance = dVariance;
            oRes = std::move(oDimList);
        }
    }
    int nSize = (int)(oRes.size());
    auto oBegin = oRes.begin();
    std::advance(oBegin, nSize / 2);    //奇数取中间，偶数取左边的
    vecDatas.swap(oRes);
    return oBegin;
}

template <typename DATATYPE,
    int DIMENSION /*= 3*/>
    void KDTree<DATATYPE, DIMENSION>::innerRebuildTree(nodeType*& pNode, std::list<nodeType*>& vecDatas)
{
    if (!vecDatas.empty())
    {
         int nDimension = -1;
         auto oIter = splitNodesByVariance(vecDatas, nDimension);
         if (oIter != vecDatas.end())
         {
             pNode = *oIter;
             pNode->m_nSplit = nDimension;
             pNode->m_pLeftTree = nullptr;
             pNode->m_pRightTree = nullptr;
             std::list<nodeType*> oLeft, oRight;
             oLeft.insert(oLeft.end(), vecDatas.begin(), oIter);
             ++oIter;
             oRight.insert(oRight.end(), oIter, vecDatas.end());
             innerRebuildTree(pNode->m_pLeftTree, oLeft);
             innerRebuildTree(pNode->m_pRightTree, oRight);
         }
    }
}

template <typename DATATYPE,
    int DIMENSION /*= 3*/>
    nodeType* KDTree<DATATYPE, DIMENSION>::addNode(DATATYPE data[DIMENSION])
{
    m_lock.lockForWrite();
    ++m_nModifiedCount;
    if (m_nModifiedCount > m_nRebuildThreshold)
    {
        //重建树，根据维度的方差让树趋近于平衡
        m_nModifiedCount = 0;
        int nCount = 0;
        std::list<nodeType*> vecDatas = std::move(traverseTree());
        if (!vecDatas.empty())
        {
            rebuildTree(vecDatas);
            free(vecDatas);
        }
    }
    nodeType *pNode = new nodeType;
    memcpy(pNode->m_data, data, DIMENSION);
    //递归插入
    if (m_pRoot == nullptr)
    {
        m_pRoot = pNode;
    }
    else
    {
        innerAddNode(m_pRoot, pNode);
    }
    m_lock.unlock();
    return pNode;
}

template <typename DATATYPE,
    int DIMENSION /*= 3*/>
    void KDTree<DATATYPE, DIMENSION>::free(std::list<DATATYPE*>& datas)
{
    for (auto iter = datas.begin(); iter != datas.end(); ++iter)
    {
        DATATYPE* pDatas = *iter;
        delete []pDatas;
    }
    datas.swap(std::list<DATATYPE*>());
}

template <typename DATATYPE,
    int DIMENSION /*= 3*/>
    void KDTree<DATATYPE, DIMENSION>::rebuildTree(std::list<nodeType*>& vecDatas)
{
    m_lock.lockForWrite();
    innerRebuildTree(m_pRoot, vecDatas);
    m_lock.unlock();
}

template <typename DATATYPE,
    int DIMENSION /*= 3*/>
    void KDTree<DATATYPE, DIMENSION>::innerTraverse(std::list<nodeType*>& vecDatas, nodeType* pNode, nodeType* pExceptNode)
{
    if (pNode)
    {
        nodeType* pLeft = pNode->m_pLeftTree;
        nodeType* pRight = pNode->m_pRightTree;
        if (pNode != pExceptNode)
        {
            vecDatas.push_back(pNode);
        }
        innerTraverse(vecDatas, pLeft, pExceptNode);
        innerTraverse(vecDatas, pRight, pExceptNode);
    }
}

template <typename DATATYPE,
    int DIMENSION /*= 3*/>
     std::list<nodeType*> KDTree<DATATYPE, DIMENSION>::traverseTree(nodeType* pNodeExcept /*= nullptr*/)
{
    std::list<nodeType*> res;
    int nValidCount = m_nNodeCount;
    if (pNodeExcept)
    {
        --nValidCount;
    }
    if (nValidCount > 0)
    {
        innerTraverse(res, m_pRoot, pNodeExcept);
    }
    return res;
}


template <typename DATATYPE,
    int DIMENSION /*= 3*/>
    nodeType* KDTree<DATATYPE, DIMENSION>::innerFindNode(nodeType* pNode, DATATYPE data[DIMENSION])
{
    if (pNode == nullptr)
    {
        return nullptr;
    }
    if (sameNode(pNode, data, m_dEpsilon))
    {
        return pNode;
    }
    else
    {
        if (data[pNode->m_nSplit] < pNode->m_data[pNode->m_nSplit])
        {
            return innerFindNode(pNode->m_pLeftTree, data);
        }
        else
        {
            return innerFindNode(pNode->m_pRightTree, data);
        }
    }
}

template <typename DATATYPE,
    int DIMENSION /*= 3*/>
    nodeType* KDTree<DATATYPE, DIMENSION>::findNode(DATATYPE data[DIMENSION])
{
    m_lock.lockForRead();
    nodeType* pRes = innerFindNode(m_pRoot, data);;
    m_lock.unlock();
    return pRes;
}

template <typename DATATYPE,
    int DIMENSION /*= 3*/>
    void KDTree<DATATYPE, DIMENSION>::removeNode(DATATYPE data[DIMENSION])
{
    m_lock.lockForWrite();
    nodeType* pNode = findNode(data);
    if (pNode != nullptr)
    {
        if (pNode->isLeaf() && !pNode->isRoot())
        {
            nodeType*& pLeaf = pNode->m_pParent->m_pLeftTree == pNode ?
                pNode->m_pParent->m_pLeftTree : pNode->m_pParent->m_pRightTree;
            pLeaf = nullptr;
        }
        else
        {
            //删除一个中间节点或者根节点，代价太高，直接重造一个树
            //其实KD树不太适合动态插入删除
            std::list<nodeType*> vecDatas = std::move(traverseTree(pNode));
            if (!vecDatas.empty())
            {
                rebuildTree(vecDatas);
                m_nNodeCount = vecDatas.size();
                free(vecDatas);
            }
        }
        delete pNode;
    }
    m_lock.unlock();
}

template <typename DATATYPE,
    int DIMENSION /*= 3*/>
    void KDTree<DATATYPE, DIMENSION>::innerAddNode(nodeType* pParent, nodeType* pNode)
{
    if (pParent && pNode)
    {
        //不能追加相同的节点
        if (sameNode(pParent, pNode, m_dEpsilon))
        {
            return;
        }
        nodeType** pNextNode = nullptr;
        if (pNode->m_data[pParent->m_nSplit]
            < pParent->m_data[pParent->m_nSplit])
        {
            pNextNode = &pParent->m_pLeftTree;
        }
        else
        {
            pNextNode = &pParent->m_pRightTree;
        }
        if (pNextNode)
        {
            innerAddNode(*pNextNode, pNode);
        }
        else
        {
            //以父的下一维作为该节点的切分维
            pNode->m_pParent = pParent;
            *pNextNode = pNode;
            pNode->m_nSplit = pParent->m_nSplit;
            ++(pNextNode->m_nSplit);
            if (pNode->m_nSplit >= DIMENSION)
            {
                pNode->m_nSplit = 0;
            }
            ++m_nNodeCount;
        }
    }
}

template <typename DATATYPE,
    int DIMENSION /*= 3*/>
    void KDTree<DATATYPE, DIMENSION>::freeNode(nodeType* pNode)
{
    if (pNode)
    {
        nodeType* pLeft = pNode->m_pLeftTree;
        nodeType* pRight = pNode->m_pRightTree;
        freeNode(pLeft);
        freeNode(pRight);
        delete pLeft;
        delete pRight;
        pNode->m_pLeftTree = nullptr;
        pNode->m_pRightTree = nullptr;
    }
}

template <typename DATATYPE,
    int DIMENSION /*= 3*/>
    void KDTree<DATATYPE, DIMENSION>::clearTree()
{
    m_lock.lockForWrite();
    if (m_pRoot)
    {
        freeNode(m_pRoot);
    }
    m_lock.unlock();
}

template <typename DATATYPE,
    int DIMENSION>
    KDTree<DIMENSION, DATATYPE>::~KDTree()
{
    clearTree();
}

template <typename DATATYPE,
    int DIMENSION>
    KDTree<DIMENSION, DATATYPE>::KDTree(double dEpsilon, 
    int nRebuildThreshold)
    :m_pRoot(nullptr), m_nRebuildThreshold(nRebuildThreshold), m_dEpsilon(fabs(dEpsilon)), m_nNodeCount(0)
{

}

