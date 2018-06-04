#pragma once
#include <QThread>
#include "GMath/GBox2.h"
#include "Geometry/Body.h"
#include "Geometry/Polygon.h"
#include "GMath/GVec2.h"
#include <QList>
#include <QSet>
#include <QObject>
#include <QEvent>
#include <QMutex>
#include "ViewManager/GViewer.h"
#include "ViewCore/GGroupNodes.h"
#include "ViewCore/GVisualNodes.h"
#include "GMath/Interval.h"
#include "GMCommon/GMPEDOBoxRtree.h"
#include "RRTAlgorithm/RRTConnectAlg.h"
#include "Algorithm/algmesh.h"

class RRTRasterizedMatrixNode;
class RRTRasterizedMatrix;
struct GTJRouteplanModel
{
    ggp::CBox2d m_rangeBox;
    QList<ggp::CBodyPtr> m_obstacles;
    QList<ggp::CCurve2dPtr> m_curves;
    GMPEDOBoxRtree<ggp::CPolygonPtr> m_polygonTree;
};

//简单的人工势场
struct GTJRoutePlanObstaclePotentialField
{
    ggp::CIntervald m_range;
    ggp::CCurve2dPtr m_curve;
    ggp::CBox2d m_rangeBox;
};

struct GTJRoutePlanArtificialPotentialField
{
public:
    void add(GTJRoutePlanObstaclePotentialField* pOPF);
    double weight(ggp::CBox3d& oBox);
public:
    QList<GTJRoutePlanObstaclePotentialField*> m_obstaclePotentials;
    GMPEDOBoxRtree<GTJRoutePlanObstaclePotentialField*> m_boxTree;
    ~GTJRoutePlanArtificialPotentialField();
};

class GTJRouteplan;
class GTJRouteplanThread : public QThread
{
    Q_OBJECT
    friend class GTJRouteplan;
public:
    GTJRouteplanThread(RRTRasterizedMatrix* pSharedMatrix,
        QList<ggp::CBodyPtr>& obstacles,
        double boxSize,
        double floorHeight,
        ggp::CVector2d& oStartPt,
        ggp::CVector2d& oEndPt,
        GTJRouteplan* pReceiver,
        int nNeighborRange = 1);
    virtual ~GTJRouteplanThread();
public:
    bool isDone();
    RRTConnectSolution* route();
protected:
    virtual void run();
private:
    void setDone();
private:
    RRTRasterizedMatrix* m_matrix;
    RRTConnectAlg* m_pAlg;
private:
    ggp::CVector2d m_startPt;
    ggp::CVector2d m_endPt;
    GTJRouteplan* m_eventReciver;
    QMutex m_mutex;
    bool m_bDone;
    double m_dFloorHeight;
    RRTConnectSolution* m_solution;
};


class GTJPathFoundEvent : public QEvent
{
public:
    GTJPathFoundEvent(QEvent::Type oType) : QEvent(oType) {}
public:
    QList<RRTRasterizedMatrixNode*> path;
};

struct GTJRoutePlanStartEndPoint
{
    ggp::CPolygonPtr oStartPolygon;
    QList<ggp::CPolygonPtr> oExits;
    QList<ggp::CBox2d> oGuideAreas;
};
class GTJRouteplan : public QObject
{
    Q_OBJECT
public:
    GTJRouteplan(GTJRouteplanModel* pInputModel,
        double dBoxSize,
        double dFloorHeight,
        GTJRoutePlanStartEndPoint& oStartEndInfo,
        int nNeighborRange);
    ~GTJRouteplan(void);
public:
    RRTConnectSolution* routeplan();
public:
    virtual bool event(QEvent* e);
private:
    void prepare(GTJRouteplanModel* pInputModel,
        double dBoxSize,
        double dFloorHeight,
        GTJRoutePlanStartEndPoint& oStartEndInfo,
        int nNeighborRange);
    bool hasThreadDone();
    bool noThreadRunning();
    RRTConnectSolution* pickRoute();
    GTJRoutePlanArtificialPotentialField* artifitialPotentialField(GTJRouteplanModel* pInputModel);
    GMPEDOBoxRtree<ggp::CBodyPtr>* obstacleRTree(GTJRouteplanModel* pInputModel);
private:
    ggp::CVector2d startPoint(GMPEDOBoxRtree<ggp::CBodyPtr>* pObstacleTree, GTJRoutePlanStartEndPoint& oStartEndInfo);
    QList<ggp::CVector2d> endPoints(GMPEDOBoxRtree<ggp::CBodyPtr>* pObstacleTree, GTJRoutePlanStartEndPoint& oStartEndInfo);
    ggp::CVector2d findObstacleFreePoint(QSet<RRTRasterizedMatrixNode*> oNodes,
        GMPEDOBoxRtree<ggp::CBodyPtr>* pObstacleTree,
        ggp::CPolygonPtr pConstraintPoly);
private:
    QSet<GTJRouteplanThread*> m_threads;
    RRTRasterizedMatrix* m_matrix;
    bool m_bDone;
    ggp::CVector2d m_startPt;
};

class GTJRoutePlaneVisualGroupNode
{
public:
    static GTJRoutePlaneVisualGroupNode* globalInstance();
public:
    void uninstall(ggp::CViewer* pViewer);
    void install(ggp::CViewer* pViewer, 
        QList<ggp::CVisualNodePtr>& visualNodes);
private:
    GTJRoutePlaneVisualGroupNode();
    ~GTJRoutePlaneVisualGroupNode();
    ggp::CRefPtr<ggp::CGroupNode> m_rootNode;
};

class GTJRouteRander
{
public:
    GTJRouteRander(ggp::CViewer* pViewer);
    ~GTJRouteRander() {}
public:
    void randerRoute(RRTConnectSolution* pRoute, double dElev);
private:
    ggp::CVisualNode* drawStartTag(ggp::CVector3d oAnchor,
        ggp::CMeshParameter& oMesh);
    ggp::CVisualNode* drawCurve(ggp::CCurve2dPtr pCurve,
        ggp::CMeshParameter& oMesh,
        double dElev);
    ggp::CVisualNode* meshPolygon(ggp::CPolygon* pPoly,
        ggp::CMeshParameter& oMesh,
        double dElev);
private:
    ggp::CViewer* m_pViewer;
};

