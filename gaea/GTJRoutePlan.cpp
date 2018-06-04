#include "GTJRouteplan.h"
#include "RRTAlgorithm/RRTRasterizedMatrix.h"
#include "ViewCore/GStateSet.h"
#include "Algorithm/algdistance.h"
#include "Algorithm/algpositionJudge.h"
#include "Algorithm/alggeobuilder.h"

static const QEvent::Type c_enPathFoundEventType = (QEvent::Type)QEvent::registerEventType();
static const int c_nMaximumWaitingTimeCount = 2 * 60;    //2min
static const double g_NotANumber = std::numeric_limits<double>::quiet_NaN();

GTJRouteplan::GTJRouteplan(GTJRouteplanModel* pInputModel,
    double dBoxSize,
    double dFloorHeight,
    GTJRoutePlanStartEndPoint& oStartEndInfo,
    int nNeighborRange) : m_bDone(false)
{
    prepare(pInputModel, dBoxSize, dFloorHeight, oStartEndInfo, nNeighborRange);
}


GTJRouteplan::~GTJRouteplan(void)
{
    qDeleteAll(m_threads);
    delete m_matrix;
    m_matrix = nullptr;
}

RRTConnectSolution* GTJRouteplan::routeplan()
{
    for (auto iter = m_threads.begin(); iter != m_threads.end(); ++iter)
    {
        GTJRouteplanThread* pThread = *iter;
        pThread->start();
    }
    int nWaitingCount = 0;
    while (nWaitingCount < 10)
    {
        if (hasThreadDone() || noThreadRunning())
        {
            break;
        }
        else
        {
            Sleep(1000);
        }
        ++nWaitingCount;
    }
    return pickRoute();
}

bool GTJRouteplan::event(QEvent* e)
{
    return QObject::event(e);
}

void GTJRouteplan::prepare(GTJRouteplanModel* pInputModel,
    double dBoxSize,
    double dFloorHeight,
    GTJRoutePlanStartEndPoint& oStartEndInfo,
    int nNeighborRange)
{
    GMPEDOBoxRtree<void*> oGuideAreaTree;
    for (auto iter = oStartEndInfo.oGuideAreas.begin(); iter != oStartEndInfo.oGuideAreas.end(); ++iter)
    {
        ggp::CBox2d& oBox = *iter;
        oGuideAreaTree.insert(nullptr, ggp::CBox3d(ggp::CVector3d(oBox.MinPt(), 0.0),
            ggp::CVector3d(oBox.MaxPt(), 0.0)));
    }

    //1. 创建势场
    GTJRoutePlanArtificialPotentialField* pAPF = artifitialPotentialField(pInputModel);
    //2. 栅格化
    ggp::CBox2d oRangeBox = pInputModel->m_rangeBox;
    ggp::CVector2d oBoxSize = oRangeBox.GetSize();
    int nRowCount = ceil(oBoxSize.Y / dBoxSize);
    int nColCount = ceil(oBoxSize.X / dBoxSize);
    m_matrix = new RRTRasterizedMatrix(nRowCount, nColCount);
    for (int i = 0 ; i < nRowCount; ++i)
    {
        double dMaxY = oRangeBox.MaxPt().Y - (i * dBoxSize);
        double dMinY = dMaxY - dBoxSize;
        for (int j = 0 ; j < nColCount; ++j)
        {
            double dMinX = oRangeBox.MinPt().X + (j * dBoxSize);
            double dMaxX = dMinX + dBoxSize;
            ggp::CBox2d oNodeBox(ggp::CVector2d(dMinX, dMinY),
                ggp::CVector2d(dMaxX, dMaxY));
            ggp::CBox3d oFilterBox(ggp::CVector3d(oNodeBox.MinPt(), -DBL_MAX),
                ggp::CVector3d(oNodeBox.MaxPt(), DBL_MAX));
            double dWeight = pAPF->weight(oFilterBox);
            std::set<void*> oGuideAreas = std::move(oGuideAreaTree.search(oFilterBox));
            if (!oGuideAreas.empty())
            {
                dWeight += 500 * oGuideAreas.size();    //引导区域权重增加
            }
            RRTRasterizedMatrixNode* pNode = new RRTRasterizedMatrixNode(oNodeBox, dWeight);
            m_matrix->set(pNode);
        }
    }
    delete pAPF;
    GMPEDOBoxRtree<ggp::CBodyPtr>* pObstacleTree = std::move(obstacleRTree(pInputModel)); 
    m_startPt = startPoint(pObstacleTree, oStartEndInfo);
    QList<ggp::CVector2d> oEndPoints = std::move(endPoints(pObstacleTree, oStartEndInfo));
    delete pObstacleTree;
    if (ggp::is_nan(m_startPt.X)
        || oEndPoints.isEmpty())
    {
        return;
    }
    //3.  根据出口创建线程
    for (auto iter = oEndPoints.begin(); iter != oEndPoints.end(); ++iter)
    {
        GTJRouteplanThread* pThread = new GTJRouteplanThread(m_matrix,
            pInputModel->m_obstacles,
            dBoxSize,
            dFloorHeight,
            m_startPt,
            *iter,
            this,
            nNeighborRange);
        m_threads.insert(pThread);
    }
}

bool GTJRouteplan::hasThreadDone()
{
    bool bRes = false;
    for (auto iter = m_threads.begin(); iter != m_threads.end(); ++iter)
    {
        if ((*iter)->isDone())
        {
            bRes = true;
            break;
        }
    }
    return bRes;
}

bool GTJRouteplan::noThreadRunning()
{
    bool bRes = true;
    for (auto iter = m_threads.begin(); iter != m_threads.end(); ++iter)
    {
        if ((*iter)->isRunning())
        {
            bRes = false;
            break;
        }
    }
    return bRes;
}

RRTConnectSolution* GTJRouteplan::pickRoute()
{
    RRTConnectSolution* pRes = nullptr;
    double dShortest = DBL_MAX;
    for (auto iter = m_threads.begin(); iter != m_threads.end(); ++iter)
    {
        GTJRouteplanThread* pThread = *iter;
        if (pThread->isDone())
        {
            RRTConnectSolution* pPath = pThread->route();
            double dPathLength = pPath->length();
            if (ggp::IsLessThan(dPathLength, dShortest, ggp::g_DistEpsilon))
            {
                pRes = pPath;
                pRes->m_startPt = m_startPt;
                dShortest = dPathLength;
            }
        }
        else
        {
            if (pThread->isRunning())
            {
                pThread->m_pAlg->abort();
                pThread->wait();
            }
        }
    }
    return pRes;
}

GTJRoutePlanArtificialPotentialField* GTJRouteplan::artifitialPotentialField(GTJRouteplanModel* pInputModel)
{
    GTJRoutePlanArtificialPotentialField* pRes = new GTJRoutePlanArtificialPotentialField();
    for (auto iter = pInputModel->m_curves.begin(); iter != pInputModel->m_curves.end(); ++iter)
    {
        ggp::CCurve2dPtr pCurve = *iter;
        double dOffset = RRTConnectAlg::s_dPotentialFieldRange;
        if (pCurve->Type() == Arc2dType)
        {
            ggp::CArc2d* pArc = dynamic_cast<ggp::CArc2d*>(pCurve.get());
            if (ggp::IsLessEqualThan(pArc->Radius(), dOffset, ggp::g_DistEpsilon))
            {
                dOffset = pArc->Radius() - 10.0;
            }
        }
        ggp::CBox2d oBox = pCurve->Box();
        ggp::CCurve2dPtr pSideCurve = pCurve->Clone();
        pSideCurve->Offset(dOffset);
        oBox.MergeBox(pSideCurve->Box());
        pSideCurve = pCurve->Clone();
        pSideCurve->Offset(-dOffset);
        oBox.MergeBox(pSideCurve->Box());
        GTJRoutePlanObstaclePotentialField* pOPF = new GTJRoutePlanObstaclePotentialField;
        pOPF->m_curve = pCurve;
        pOPF->m_rangeBox = oBox;
        pOPF->m_range = pCurve->GetRange();
        pRes->add(pOPF);
    }
    return pRes;
}

GMPEDOBoxRtree<ggp::CBodyPtr>* GTJRouteplan::obstacleRTree(GTJRouteplanModel* pInputModel)
{
    GMPEDOBoxRtree<ggp::CBodyPtr>* pRes = new GMPEDOBoxRtree<ggp::CBodyPtr>;
    for (auto iter = pInputModel->m_obstacles.begin(); iter != pInputModel->m_obstacles.end(); ++iter)
    {
        pRes->insert(*iter, (*iter)->Box());
    }
    return pRes;
}

ggp::CVector2d GTJRouteplan::startPoint(GMPEDOBoxRtree<ggp::CBodyPtr>* pObstacleTree, 
    GTJRoutePlanStartEndPoint& oStartEndInfo)
{
    ggp::CBox2d oStartPolygonBox = oStartEndInfo.oStartPolygon->Box();
    QSet<RRTRasterizedMatrixNode*> oStarts = std::move(m_matrix->search(oStartPolygonBox));
    return findObstacleFreePoint(oStarts, pObstacleTree, oStartEndInfo.oStartPolygon);
}

QList<ggp::CVector2d> GTJRouteplan::endPoints(GMPEDOBoxRtree<ggp::CBodyPtr>* pObstacleTree, 
    GTJRoutePlanStartEndPoint& oStartEndInfo)
{
    QList<ggp::CVector2d> oRes;
    for (auto iter = oStartEndInfo.oExits.begin(); iter != oStartEndInfo.oExits.end(); ++iter)
    {
        ggp::CPolygonPtr pExitPoly = *iter;
        QSet<RRTRasterizedMatrixNode*> oEnds = std::move(m_matrix->search(pExitPoly->Box()));
        ggp::CVector2d oResPnt = findObstacleFreePoint(oEnds, pObstacleTree, pExitPoly);
        if (!ggp::is_nan(oResPnt.X))
        {
            oRes.push_back(oResPnt);
        }
    }
    return oRes;
}

ggp::CVector2d GTJRouteplan::findObstacleFreePoint(QSet<RRTRasterizedMatrixNode*> oNodes, 
    GMPEDOBoxRtree<ggp::CBodyPtr>* pObstacleTree,
    ggp::CPolygonPtr pConstraintPoly)
{
    auto boundingBox2Polygon = [&] (ggp::CBox2d& oBox) ->ggp::CPolygon*
    {
        ggp::CPolygon* pRes = new ggp::CPolygon;
        ggp::CVector2d oCorners[4];
        oBox.GetCorners(oCorners);
        ggp::CLoop* pLoop = new ggp::CLoop(oCorners, 4);
        pLoop->MakeClosed();
        if (pLoop->ClockSign() == csClockwise)
        {
            pLoop->Reverse();
        }
        pRes->AddLoop(pLoop);
        return pRes;
    };

    ggp::CVector2d oRes(g_NotANumber, g_NotANumber);
    ggp::CVector2d oTiny(10, 10);
    CPositionJudge oPJ;
    for (auto iter = oNodes.begin(); iter != oNodes.end(); ++iter)
    {
        bool bValid = true;
        if (pConstraintPoly)
        {
            ggp::CPolygonPtr pPoly = boundingBox2Polygon((*iter)->m_boundingBox);
            EnPolygonPolygonPosition oPos = oPJ.GetPolygonPosition(pPoly.get(), pConstraintPoly.get());
            bValid = oPos != PP_SEPARATION && oPos != PP_EXTERNAL_TANGENT;
        }
        if (bValid)
        {
            ggp::CVector2d oPnt = (*iter)->m_boundingBox.CenterPt();
            ggp::CBox3d oFilterBox(ggp::CVector3d(oPnt - oTiny, -DBL_MAX),
                ggp::CVector3d(oPnt + oTiny, DBL_MAX));
            std::set<ggp::CBodyPtr> oBodies = std::move(pObstacleTree->search(oFilterBox));
            EnCurveBodyPosition oPos;
            ggp::CLine3d oLine(ggp::CVector3d(oPnt, 0.0),
                ggp::CVector3d(oPnt, 1.0));
            oLine.Extend(1e6, true);
            oLine.Extend(1e6, false);
            bool bObstacleFree = true;
            for (auto bodyIter = oBodies.begin(); bodyIter != oBodies.end(); ++bodyIter)
            {
                oPos = oPJ.GetCurveBodyPosition(&oLine, (*bodyIter).get());
                if (oPos == CP_INTERSECT || oPos == CP_IN || oPos == CP_ON
                    || oPos == CP_INTERNAL_TANGENT || oPos == CP_EXTERNAL_TANGENT)
                {
                    bObstacleFree = false;
                    break;
                }
            }
            if (bObstacleFree)
            {
                return oPnt;
            }
        }
    }
    return oRes;
}

GTJRouteplanThread::GTJRouteplanThread(RRTRasterizedMatrix* pSharedMatrix, 
    QList<ggp::CBodyPtr>& obstacles, 
    double boxSize, 
    double floorHeight,
    ggp::CVector2d& oStartPt,
    ggp::CVector2d& oEndPt,
    GTJRouteplan* pReceiver,
    int nNeighborRange /*= 1*/) : m_eventReciver(pReceiver), m_matrix(pSharedMatrix), m_bDone(false), m_dFloorHeight(floorHeight)
{
    m_pAlg = new RRTConnectAlg(pSharedMatrix,
        obstacles,
        boxSize,
        floorHeight,
        nNeighborRange);
    m_startPt = oStartPt;
    m_endPt = oEndPt;
}

GTJRouteplanThread::~GTJRouteplanThread()
{
    delete m_pAlg;
}

bool GTJRouteplanThread::isDone()
{
    bool bRes = false;
    m_mutex.lock();
    bRes = m_bDone;
    m_mutex.unlock();
    return bRes;
}

RRTConnectSolution* GTJRouteplanThread::route()
{
    return m_solution;
}

void GTJRouteplanThread::run()
{
    const DWORD MS_VC_EXCEPTION = 0x406D1388;  
#pragma pack(push,8)  
    typedef struct tagTHREADNAME_INFO  
    {  
        DWORD dwType; // Must be 0x1000.  
        LPCSTR szName; // Pointer to name (in user addr space).  
        DWORD dwThreadID; // Thread ID (-1=caller thread).  
        DWORD dwFlags; // Reserved for future use, must be zero.  
    } THREADNAME_INFO;  
#pragma pack(pop)  
    THREADNAME_INFO info;  
    info.dwType = 0x1000;  
    info.szName = "GTJRouteplanThread";  
    info.dwThreadID = ::GetCurrentThreadId();  
    info.dwFlags = 0;  
#pragma warning(push)  
#pragma warning(disable: 6320 6322)  
    RaiseException(MS_VC_EXCEPTION, 0, sizeof(info) / sizeof(ULONG_PTR), (ULONG_PTR*)&info);
#pragma warning(pop)  

    ggp::CVector2d oExpand(1.0, 1.0);
    QSet<RRTRasterizedMatrixNode*> oStartSeeds = std::move(m_matrix->search(ggp::CBox2d((m_startPt - oExpand),
        (m_startPt + oExpand))));
    QSet<RRTRasterizedMatrixNode*> oEndSeeds = std::move(m_matrix->search(ggp::CBox2d((m_endPt - oExpand),
        (m_endPt + oExpand))));
    if (oStartSeeds.empty()
        || oEndSeeds.empty())
    {
        return;
    }

    RRTRasterizedMatrixNode* pStartNode = *oStartSeeds.begin();
    RRTRasterizedMatrixNode* pEndNode = *oEndSeeds.begin();
    m_solution = m_pAlg->routeplan(pStartNode, pEndNode);
    if (m_solution && !m_solution->m_curves.empty())
    {
        setDone();
    }
}

void GTJRouteplanThread::setDone()
{
    m_mutex.lock();
    m_bDone = true;
    m_mutex.unlock();
}

GTJRouteRander::GTJRouteRander(ggp::CViewer* pViewer)
    :m_pViewer(pViewer)
{

}

static ggp::CStateSet* pSharedState = nullptr;
void GTJRouteRander::randerRoute(RRTConnectSolution* pRoute, double dElev)
{
    if (pSharedState == nullptr)
    {
        pSharedState = new ggp::CStateSet;
        pSharedState->Ref();
    }
    pSharedState->SetDiffuse(0, 205, 205, 200);
    pSharedState->SetLineWidth(2);
    pSharedState->SetDepthTestEnabled(false);
    pSharedState->SetLightModelTwoSided(true);
    pSharedState->SetFaceMode(ggp::FM_FRONT_AND_BACK);
    GTJRoutePlaneVisualGroupNode::globalInstance()->uninstall(m_pViewer);
    if (pRoute && !pRoute->m_curves.empty())
    {
        ggp::CMeshParameter oMesh;
        QList<ggp::CVisualNodePtr> oVisualNodes;
        //在起点上画个圆
        oVisualNodes.push_back(drawStartTag(ggp::CVector3d(pRoute->m_startPt, dElev), oMesh));
        for (auto iter = pRoute->m_curves.begin(); iter != pRoute->m_curves.end(); ++iter)
        {
            oVisualNodes.push_back(drawCurve(*iter, oMesh, dElev));
        }
        GTJRoutePlaneVisualGroupNode::globalInstance()->install(m_pViewer, oVisualNodes);
    }
}

ggp::CVisualNode* GTJRouteRander::drawStartTag(ggp::CVector3d oAnchor, 
    ggp::CMeshParameter& oMesh)
{
    ggp::CPolygonPtr pPoly = new ggp::CPolygon;
    ggp::CLoop* pLoop = new ggp::CLoop;
    ggp::CArc2d* pArc = new ggp::CArc2d(oAnchor.Vec2(), 200, 0, M_PI * 2, 1);
    pLoop->AddCurve(pArc);
    pPoly->AddLoop(pLoop);
    return meshPolygon(pPoly.get(), oMesh, oAnchor.Z + 20);
}

ggp::CVisualNode* GTJRouteRander::drawCurve(ggp::CCurve2dPtr pCurve,
    ggp::CMeshParameter& oMesh,
    double dElev)
{
    ggp::CPolygonPtr pPoly = GPolygonBuilder::CreatePolygon(pCurve.get(), 20, 120);
    return meshPolygon(pPoly.get(), oMesh, dElev);
}

ggp::CVisualNode* GTJRouteRander::meshPolygon(ggp::CPolygon* pPoly, 
    ggp::CMeshParameter& oMesh,
    double dElev)
{
    ggp::CVec3fList oVertexes;
    ggp::CVec3fList oNormals;
    std::vector<unsigned int> oIndex;
    oMesh.PolygonToMesh(pPoly, oVertexes, oIndex, dElev);
    ggp::CVisualNode* pRes = new ggp::CVisualNode;
    ggp::CPrimitiveRenderable* pRender = new ggp::CPrimitiveRenderable();
    pRender->SetPrimitiveType(PT_TRIANGLES);
    pRender->SetVertexArray(oVertexes.data(), oVertexes.size());
    pRender->SetIndexArray(oIndex.data(), oIndex.size());
    pRender->SetStateSet(pSharedState);
    pRender->SetPriority(-1000);
    pRes->AddRenderable(pRender);
    pRes->AddRenderable(pRender);
    return pRes;
}

GTJRoutePlaneVisualGroupNode* GTJRoutePlaneVisualGroupNode::globalInstance()
{
    static GTJRoutePlaneVisualGroupNode m_instance;
    return &m_instance;
}

void GTJRoutePlaneVisualGroupNode::uninstall(ggp::CViewer* pViewer)
{
    int nChildCount = m_rootNode->NumChildren();
    for (int i = nChildCount - 1 ; i >= 0; --i)
    {
        ggp::CSceneNode* pChild = m_rootNode->GetChild(i);
        m_rootNode->RemoveChild(pChild);
    }
    pViewer->Scene()->GetRootNode()->RemoveChild(m_rootNode.get());
    pViewer->Scene()->SetTempDirty();
}

void GTJRoutePlaneVisualGroupNode::install(ggp::CViewer* pViewer,
    QList<ggp::CVisualNodePtr>& visualNodes)
{
    for (auto iter = visualNodes.begin(); iter != visualNodes.end(); ++iter)
    {
        m_rootNode->AddChild((*iter).get());
    }
    pViewer->Scene()->GetRootNode()->AddChild(m_rootNode.get());
    pViewer->Scene()->SetDirty();
}

GTJRoutePlaneVisualGroupNode::GTJRoutePlaneVisualGroupNode()
{
    m_rootNode = new ggp::CGroupNode;
}

GTJRoutePlaneVisualGroupNode::~GTJRoutePlaneVisualGroupNode()
{

}

void GTJRoutePlanArtificialPotentialField::add(GTJRoutePlanObstaclePotentialField* pOPF)
{
    m_obstaclePotentials.push_back(pOPF);
    ggp::CBox2d oFilterBox = pOPF->m_rangeBox;
    m_boxTree.insert(pOPF, ggp::CBox3d(ggp::CVector3d(oFilterBox.MinPt(), 0.0),
        ggp::CVector3d(oFilterBox.MaxPt(), 0.0)));
}

double GTJRoutePlanArtificialPotentialField::weight(ggp::CBox3d& oBox)
{
    double dRes = RRTConnectAlg::s_dPotentialFieldRange;
    std::set<GTJRoutePlanObstaclePotentialField*> oOPFs = std::move(m_boxTree.search(oBox));
    ggp::CVector2d oCenter = oBox.Box2().CenterPt();
    for (auto iter = oOPFs.begin(); iter != oOPFs.end(); ++iter)
    {
        GTJRoutePlanObstaclePotentialField* pOPF = *iter;
        if (pOPF->m_range.Contain(pOPF->m_curve->GetNearestT(oCenter)))
        {
            CCurve2dMindist cm(pOPF->m_curve.get(), true);
            cm.SetQueryPoint(oCenter);
            double dDis = cm.GetDistance();
            if (dDis < RRTConnectAlg::s_dPotentialFieldRange)
            {
                dRes -= (RRTConnectAlg::s_dPotentialFieldRange - dDis) * RRTConnectAlg::s_dWeightDecay;
            }
        }
    }
    return dRes;
}

GTJRoutePlanArtificialPotentialField::~GTJRoutePlanArtificialPotentialField()
{
    m_boxTree.removeAll();
    qDeleteAll(m_obstaclePotentials);
}
