#include "GTJEditEmergencyExitCmdState.h"
#include "GMModel/IGMPFloor.h"
#include "Algorithm/algmesh.h"
#include "GMPCore/GMPModelViewer.h"
#include "GMPCore/GMPCmdState.h"
#include "Algorithm/algpositionJudge.h"
#include "Common/scopeguard.h"
#include "GMModel/GMPStateSetManager.h"
GMP_REGISTER_CMDSTATE(GTJEditEmergencyExitCmdState, L"GTJCMD_EDIT_EMERGENCYEXIT", CP_NORMAL);
GTJEditEmergencyExitCmdState::GTJEditEmergencyExitCmdState(void)
{

}


GTJEditEmergencyExitCmdState::~GTJEditEmergencyExitCmdState(void)
{

}

void GTJEditEmergencyExitCmdState::exec()
{
    const int c_nCurveType = 1;
    GMPSystemOptions::getInstance()->setSysOption("OptionCurveType", c_nCurveType);
    m_pCmdState->gmpService()->modelViewer()->Input(L"SetCurveType");
    QCoreApplication::processEvents();
    initialize();
    IGMPFloor* pCurFloor = m_pCmdState->gmpService()->state()->curFloor();
    IGMPHitService *const pHitService = m_pCmdState->gmpService()->hitService();
    ggp::CCoordinates3d oPlane = m_pCmdState->curWorkPlane();
    pHitService->reset(); 
    pHitService->setMode(GMPHitModes(hsmPlane | hsmWorkPlane | hsmSolid));
    pHitService->setWorkPlane(oPlane);
    acquireAndDrawExsited(pCurFloor->iD());
    QList<GMPDrawCurveInfo> oConfirmedCurves;
    ggp::CVisualNode* pOperatingVisualNode = nullptr;
    auto sampler = [&] (const GMPJigMouseKeyState &oState, 
        QList<GMPHitPointInfo>& points,
        const QList<GMPDrawCurveInfo>& curves, 
        GMPDrawCurveStatus& eDrawCurveStatus)->GMPJigStatus
    {
        QList<GMPDrawCurveInfo> oCurves;
        oCurves.append(oConfirmedCurves);
        oCurves.append(curves);
        ggp::CPolygonPtr pPoly = makePolygon(oCurves);
        if (pPoly && !pPoly->IsEmpty())
        {
            showPolygon(pOperatingVisualNode, pPoly.get());
        }
        else
        {
            CPrimitiveRenderable* pRender = dynamic_cast<ggp::CPrimitiveRenderable*>(pOperatingVisualNode->GetRenderable(0));
            if (pRender)
            {
                pRender->ClearVertexAndIndexData();
            }
        }
        return oState.Status;
    };

    auto confirm = [&](GMPJigMouseKeyState oState, QList<GMPDrawCurveInfo>& curves, bool& bAccept)->GMPJigStatus
    {
        if (curves.empty())
        {
            oConfirmedCurves.pop_back();
        }
        else
        {
            oConfirmedCurves.append(curves);
        }
        ggp::CPolygonPtr pPoly = makePolygon(oConfirmedCurves);
        if (pPoly && !pPoly->IsEmpty())
        {
            showPolygon(pOperatingVisualNode, pPoly.get());
        }
        else
        {
            if (oConfirmedCurves.size() > 1)
            {
                oConfirmedCurves.pop_back();
                bAccept = false;
            }
        }
        return oState.Status;
    };

    while (true) {
        pOperatingVisualNode = new ggp::CVisualNode;
        m_pGropNode->AddChild(pOperatingVisualNode);
        oConfirmedCurves.clear();
        QList<GMPDrawCurveInfo> oResult;
        GMPDrawCurveMode oMode(false, true, GMPDrawCurveMode::dccClosed);
        GMPDrawCurveTypes oTypes = dctAll;
        GMPPickAndDrawOptions oOption((c_DrawCurveDefOpt | pdoProjection | pdoOffsetByShift));
        GMPJigStatus oStatus = m_pCmdState->drawCurves(oResult,
            oMode, 
            &oPlane, 
            oTypes, 
            hptAll, 
            oOption, 
            nullptr, 
            sampler, 
            confirm);
        if (gjsCancel == oStatus
            || oResult.empty())
        {
            break;
        }
        if (oConfirmedCurves.isEmpty())
        {
            continue;
        }
        ggp::CPolygonPtr pPoly = makePolygon(oConfirmedCurves);
        if (!saveExit(pPoly.get(), pCurFloor->iD()))
        {
            m_pGropNode->RemoveChild(pOperatingVisualNode);
        }
    }
    finalize();
}

void GTJEditEmergencyExitCmdState::switchTo2DMode()
{
    GMPModelViewer* pModelView = m_pCmdState->gmpService()->modelViewer();
    assert(pModelView);
    if (!pModelView->is3DMode())
    {
        return;
    }
    pModelView->setIs3dMode(false);
    pModelView->setViewPoint(VP_TOP);
    pModelView->rotateScreen(m_pCmdState->gmpService()->state()->getCur2dRotateAngle());
}

void GTJEditEmergencyExitCmdState::acquireAndDrawExsited(int64_t nFloorID)
{
    GMPStateSet oState;
    oState.m_nLightClose = false;
    oState.m_nTwoSideShow = true;
    oState.m_nFillStyle = 0;
    oState.m_nLineWidth = 2;
    oState.m_nLineStyle = 0;
    oState.m_nDepthClose = true;
    oState.m_nTransparent = 100;
    oState.m_nColor = ggp::CColor(34, 117, 76).ToInteger();
    m_pSharedState = GMPStateSetManager::getViewStateSet(oState);
    ggp::CDatabase* pProjDB = m_pCmdState->gmpService()->model()->projectDB();
    ggp::CDBTable* pTable = pProjDB->GetTable(L"EmergencyExit");
    if (pTable)
    {
        ggp::CFileAddressList oExists;
        pTable->Search(pTable->FindField(L"floorID"),
            nFloorID, &oExists);
        ggp::CDBField* pPolygonFiled = pTable->GetField(L"range");
        QList<ggp::CPolygonPtr> oExitPolygons;
        for (int i = 0 ; i < oExists.GetCount(); ++i)
        {
            ggp::FileAddress* pExit = oExists.GetItem(i);
            ggp::CDBRecord* pRec = pTable->CreateRecordMap(*pExit);
            ggp::CPolygonPtr pPolygon = pPolygonFiled->GetPolygon(pRec)->Clone();
            ggp::CBox2d oBox = pPolygon->Box();
            ggp::CBox3d oFilterBox(ggp::CVector3d(oBox.MinPt(), 0.0),
                ggp::CVector3d(oBox.MaxPt(), 0.0));
            m_oExitRTree.insert(pPolygon, oFilterBox);
            oExitPolygons.push_back(pPolygon);
            delete pRec;
        }
        for (auto iter = oExitPolygons.begin(); iter != oExitPolygons.end(); ++iter)
        {
            ggp::CPolygonPtr pPoly = *iter;
            ggp::CVisualNode* pVisualNode = new ggp::CVisualNode;
            pVisualNode->SetTempNodeType(CVisualNode::TNT_ALWAYS);
            showPolygon(pVisualNode, pPoly.get());
            m_pGropNode->AddChild(pVisualNode);
        }
        m_pCmdState->gmpService()->modelViewer()->RealViewer()->Scene()->SetTempDirty(true);
    }
}

void GTJEditEmergencyExitCmdState::initialize()
{
    m_pGropNode = new ggp::CGroupNode();
    m_pCmdState->gmpService()->modelViewer()->RealViewer()->Scene()->GetRootNode()->AddChild(m_pGropNode);
}

void GTJEditEmergencyExitCmdState::finalize()
{
    m_oExitRTree.removeAll();
    m_pCmdState->gmpService()->modelViewer()->RealViewer()->Scene()->GetRootNode()->RemoveChild(m_pGropNode);
}

void GTJEditEmergencyExitCmdState::showPolygon(ggp::CVisualNode* pVisualNode, ggp::CPolygon* pPolygon)
{
    if (pVisualNode && pPolygon)
    {
        ggp::CMeshParameter oMesh;
        ggp::CVec3fList oVertexes;
        ggp::CVec3fList oNormals;
        std::vector<unsigned int> oIndex;
        oMesh.PolygonToMesh(pPolygon, oVertexes, oIndex);
        ggp::CPrimitiveRenderable* pRender = nullptr;
        if (pVisualNode->GetRenderableCount() <= 0)
        {
            pRender = new ggp::CPrimitiveRenderable();
            pVisualNode->AddRenderable(pRender);
        }
        else
        {
            pRender = dynamic_cast<ggp::CPrimitiveRenderable*>(pVisualNode->GetRenderable(0));
        }
        pRender->SetPrimitiveType(PT_TRIANGLES);
        pRender->SetVertexArray(oVertexes.data(), oVertexes.size());
        pRender->SetIndexArray(oIndex.data(), oIndex.size());
        pRender->SetStateSet(m_pSharedState);
        pRender->SetPriority(-1000);
        m_pCmdState->gmpService()->modelViewer()->RealViewer()->Scene()->SetTempDirty();
    }
}

ggp::CPolygon* GTJEditEmergencyExitCmdState::makePolygon(QList<GMPDrawCurveInfo>& oCurves)
{
    ggp::CPolygon* pRes = new ggp::CPolygon;
    if (oCurves.size() == 1)
    {
        return pRes;
    }
    ggp::CLoop* pLoop = new ggp::CLoop;
    for (auto iter = oCurves.begin(); iter != oCurves.end(); ++iter)
    {
        const GMPDrawCurveInfo& oInfo = *iter;
        auto pCurve3d = dynamic_cast<CPlaneCurve3d*>(oInfo.Curve.get());
        pLoop->AddCurve(pCurve3d->Curve2d()->Clone());
    }
    pLoop->MakeClosed();
    if (!pLoop->IsStrictlyValid())
    {
        return pRes;
    }
    if (pLoop->ClockSign() == csClockwise)
    {
        pLoop->Reverse();
    }
    pRes->AddLoop(pLoop);
    return pRes;
}

bool GTJEditEmergencyExitCmdState::saveExit(ggp::CPolygon* pPoly, int64_t nFloorID)
{
    if (pPoly)
    {
        if (checkOverlap(pPoly))
        {
            return false;
        }
        ggp::CDatabase* pDB = m_pCmdState->gmpService()->model()->projectDB();
        if (pDB)
        {
            ggp::CDBTable* pTable = pDB->GetTable(L"EmergencyExit");
            FileAddress oAddr = pTable->NewRecord();
            pTable->AddRecord(oAddr);
            ggp::CDBRecord* pRec = pTable->CreateRecordMap(oAddr);
            ggp::CDBField* pFloorIDField = pTable->GetField(L"floorID");
            ggp::CDBField* pRangeField = pTable->GetField(L"range");
            pFloorIDField->SetInt64(pRec, nFloorID);
            pRangeField->SetPolygon(pRec, pPoly);
            delete pRec;
        }
        return true;
    }
    return false;
}

bool GTJEditEmergencyExitCmdState::checkOverlap(ggp::CPolygon* pPoly)
{
    ggp::CBox2d oBox = pPoly->Box();
    ggp::CBox3d oFilterBox(ggp::CVector3d(oBox.MinPt(),
        -DBL_MAX), ggp::CVector3d(oBox.MaxPt(), DBL_MAX));
    std::set<ggp::CPolygonPtr>& oIntPolygons = std::move(m_oExitRTree.search(oFilterBox));
    ggp::CPositionJudge oPJ;
    for (auto iter = oIntPolygons.begin(); iter != oIntPolygons.end(); ++iter)
    {
        ggp::CPolygonPtr pPolyOnTree = *iter;
        EnPolygonPolygonPosition oPos = oPJ.GetPolygonPosition(pPoly, pPolyOnTree.get());
        if (!(oPos == PP_SEPARATION || 
            oPos == PP_EXTERNAL_TANGENT))
        {
            return true;
        }
    }
    return false;
}
