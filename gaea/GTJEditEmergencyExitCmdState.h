#pragma once
#include "GMPCore/GMPCmdStateIntf.h"
#include "GMCommon/GMPEDOBoxRtree.h"
#include "ViewCore/GSceneNode.h"
#include "ViewCore/GStateSet.h"

class GTJEditEmergencyExitCmdState : public IGMPBaseCmdState
{
public:
    GTJEditEmergencyExitCmdState(void);
    ~GTJEditEmergencyExitCmdState(void);
public:
    virtual void exec();
    virtual bool initialCustomData(int nUserData, void *pUserData) { return true; }
private:
    void switchTo2DMode();
    void acquireAndDrawExsited(int64_t nFloorID);
    void initialize();
    void finalize();
    void showPolygon(ggp::CVisualNode* pVisualNode,
        ggp::CPolygon* pPolygon);
    ggp::CPolygon* makePolygon(QList<GMPDrawCurveInfo>& oCurves);
    bool saveExit(ggp::CPolygon* pPoly, int64_t nFloorID);
    bool checkOverlap(ggp::CPolygon* pPoly);
private:
    GMPEDOBoxRtree<ggp::CPolygonPtr> m_oExitRTree;
    ggp::CGroupNode* m_pGropNode;
    ggp::CStateSet* m_pSharedState;
};

