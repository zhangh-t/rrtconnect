#include "GTJRoutePlanCmdState.h"
#include "GTJRoutePlan.h"
#include "GTJCommon/GTJEDOIteratorCreatror.h"
#include "ModelUtilities/G3dsExporter.h"
#include "GTJTypes.h"
#include "GMPCore/GMPModelViewer.h"
#include "GMModel/IGMPRelationOperator.h"
#include "Algorithm/algbool.h"
#include "Geometry/Plane.h"
#include "GMModel/IGMPProperties.h"
#include "Common/scopeguard.h"
#include "GMPCore/GMPCmdState.h"
#include "GMPCore/GMPAppInfo.h"
#include "GTFWebGaeaInterface.h"
#include "ViewCore/GImageHelper.h"
#include "ViewManager/GScreenshoter.h"
#include <QHBoxLayout>
#include <QJsonObject>
#include <QVector>
#include <QJsonDocument>
static const double c_dObjectSize = 3.0;
static const int c_nOnFireTransparent = 100;
static const int c_nNotOnFireTransparent = 0;
GMP_REGISTER_CMDSTATE(GTJRoutePlanCmdState, L"GTJCMD_ROUTE_PLAN", CP_NORMAL);
GTJRoutePlanCmdState::GTJRoutePlanCmdState(void)
    :m_pQRCode(nullptr), m_pRoutePlaneModel(nullptr), m_qrWidget(nullptr)
{

}


GTJRoutePlanCmdState::~GTJRoutePlanCmdState(void)
{

}

void GTJRoutePlanCmdState::exec()
{
    GTJRoutePlanRequestCallBack oCallBack = std::bind(&GTJRoutePlanCmdState::onRoutePlanRequest, this, std::placeholders::_1);
    m_pCom = new GTJRoutePlanWebCom(oCallBack);
    m_qrWidget = new GTJQRWidget(m_pCmdState->gmpService()->modelViewer());
    m_pCmdState->gmpService()->modelViewer()->setIs3dMode(true);
    QList<int> oVisiablElementTypes;
    oVisiablElementTypes.push_back(etWall);
    oVisiablElementTypes.push_back(gtj::etBrickWall);
    oVisiablElementTypes.push_back(etInsulatingWall);
    oVisiablElementTypes.push_back(etParapet);
    oVisiablElementTypes.push_back(etRibbonWin);
    oVisiablElementTypes.push_back(gtj::etCurtainWall);
    oVisiablElementTypes.push_back(etCustomLine);
    m_pCmdState->pushEDOVisibleSettings(oVisiablElementTypes, true);
    SCOPE_EXIT {m_pCmdState->popEDOVisibleSettings(false);};

    int64_t nCurFloorID = m_pCmdState->gmpService()->state()->curFloor()->iD();
    double dFloorBtmElev = m_pCmdState->gmpService()->state()->curFloor()->btmElev() * 1e3;
    QList<IGMPElementDrawObj*> oResult;
    GMPPickMode oMode(GMPPickMode::pmAdding, GMPPickMode::pmSingle);
    m_pCmdState->hitService()->setEDOValidFunc([] (IGMPElementDrawObj* pEDO) -> bool {
        return etDoor == pEDO->elementType();
    });
    IGMPElementDrawObj* pPrevSelected = nullptr;
    IGMPEDOSelSet* pEDOSelSet = m_pCmdState->gmpService()->state()->curEDOSelSet();
    auto removeSelect = [&pPrevSelected, &pEDOSelSet] () {
        if (pPrevSelected != nullptr)
        {
            pEDOSelSet->remove(pPrevSelected);
            pPrevSelected->state()->setSelected(false);
        }
    };
    
    auto confirm = [&] (GMPJigMouseKeyState oState, QList<GMPPickedEDO>& oPicked) -> GMPJigStatus {
        if (!oPicked.isEmpty())
        {
            removeSelect();
            pPrevSelected = oPicked.at(0).Info;
            IGMPCustomPointSolidShape* pShape = dynamic_cast<IGMPCustomPointSolidShape*>(pPrevSelected->shape());
            if (pShape)
            {
                ggp::CVector3d oAnchor(pShape->worldInsertPt(), dFloorBtmElev);
                showQR(nCurFloorID, pPrevSelected->iD(), oState.ScreenPoint, oAnchor);
            }
        }
        return oState.Status;
    };
    ggp::CViewer* pViewer = m_pCmdState->gmpService()->modelViewer()->RealViewer();
    GTJQRWidgetObserver oObserver(m_qrWidget, pViewer);
    pViewer->Attach(&oObserver);
    while (true) {
        GMPJigStatus oStatu = m_pCmdState->pickEDOs(oResult,
            oMode,
            nullptr,
            c_PickEDODefOpt | pdoIgnoreLayer,
            NULL,
            confirm);
        if (oStatu == gjsCancel || oStatu == gjsFinish)
        {
            break;
        }
    }
    removeSelect();
    pViewer->Detach(&oObserver);
    finalize();
}

void GTJRoutePlanCmdState::acquireAllEmergencyExist(QMap<IGMPElementDrawObj*, QList<IGMPElementDrawObj*>>& oExit,
    GTJRoutePlanStartEndPoint& oStartEndPoint,
    IGMPElementDrawObj* pStartEDO)
{
    IGMPFloor* pCurFloor = m_pCmdState->gmpService()->state()->curFloor();
    std::set<int> oElementSet;
    oElementSet.insert(etDoor);
    IGMPEdoIterator* pIter = m_pCmdState->gmpService()->model()->edoContnr()->createIter(pCurFloor->iD(),
        etDoor);
    IGMPRelationOperator* pRelaOpr = m_pCmdState->gmpService()->model()->oprCenter()->relaOpr();
    for (pIter->first(); !pIter->isDone(); pIter->next())
    {
        IGMPElementDrawObj* pDoor = pIter->curItem();
        if (pRelaOpr->main(pDoor) != nullptr
            || !pDoor->properties()->hasProp(L"EmergencyExit")
            || !pDoor->properties()->asBoolean(L"EmergencyExit"))
        {
            continue;
        }
        IGMPElementDrawObj* pParent = pRelaOpr->parent(pDoor);
        addEmergencyExit(pParent, pDoor, oExit);
        if (pDoor != pStartEDO)
        {
            oStartEndPoint.oGuideAreas.push_back(pDoor->shape()->box(false).Box2());
        }
    }
    delete pIter;
}

void GTJRoutePlanCmdState::addEmergencyExit(IGMPElementDrawObj* pParent,
    IGMPElementDrawObj* pDoor,
    QMap<IGMPElementDrawObj*, QList<IGMPElementDrawObj*>>& oExit)
{
    if (oExit.contains(pParent))
    {
        oExit[pParent].push_back(pDoor);
    }
    else
    {
        QList<IGMPElementDrawObj*> oDoors;
        oDoors.push_back(pDoor);
        oExit[pParent] = std::move(oDoors);
    }
}

void GTJRoutePlanCmdState::breakObstacle(IGMPCustomLineSolidShape* pShape, 
    QList<ggp::CBodyPtr>& oObstacles,
    QList<ggp::CCurve2dPtr>& oCurves,
    QMap<IGMPElementDrawObj*, QList<IGMPElementDrawObj*>>& oExit)
{
    if (pShape)
    {
        IGMPElementDrawObj* pEDO = pShape->owner();
        if (pEDO)
        {
            if (oExit.contains(pEDO))
            {
                ggp::CCurve2dPtr pWorldLine = pShape->worldLine(false);
                ggp::CBodyPtr pBody = pShape->body(false);
                std::set<obstacleCutParam> oCutParams;
                QList<IGMPElementDrawObj*>& oDoors = oExit[pEDO];
                for (auto iter = oDoors.begin(); iter != oDoors.end(); ++iter)
                {
                    IGMPElementDrawObj* pDoor = *iter;
                    IGMPCustomPointSolidShape* pDoorShape = dynamic_cast<IGMPCustomPointSolidShape*>(pDoor->shape());
                    if (pDoorShape)
                    {
                        obstacleCutParam oParam;
                        oParam.dT = pWorldLine->GetNearestT(pDoorShape->worldInsertPt());
                        oParam.dLength = pDoor->properties()->asInteger(pfnOpeningWidth) * 0.5;
                        oCutParams.insert(oParam);
                    }
                }
                breakLineObstacle(pWorldLine,
                    pBody,
                    oCutParams,
                    oObstacles,
                    oCurves);
            }
            else
            {
                oObstacles.push_back(pShape->body(false));
                oCurves.push_back(pShape->worldLine(false));
            }
        }
    }
}

void GTJRoutePlanCmdState::breakLineObstacle(ggp::CCurve2dPtr pWorldLine,
    ggp::CBodyPtr pBody,
    std::set<obstacleCutParam> oCutParams,
    QList<ggp::CBodyPtr>& oObstacles,
    QList<ggp::CCurve2dPtr>& oCurves)
{
    ggp::CCurve2dPtr pOriginWorldLine = pWorldLine->Clone();
    ggp::CBooleanOperate oBoolOpr;
    for (auto iter = oCutParams.begin(); iter != oCutParams.end()
        && pWorldLine != nullptr && pBody != nullptr; ++iter)
    {
        const obstacleCutParam& oParam = *iter;
        ggp::CIntervald oCutInterval(pWorldLine->GetNearestT(pOriginWorldLine->GetPoint(oParam.dT - oParam.dLength)), 
            pWorldLine->GetNearestT(pOriginWorldLine->GetPoint(oParam.dT + oParam.dLength)));
        ggp::CIntervald oWorldLineRange = pWorldLine->GetRange();
        if (oWorldLineRange.Contain(oCutInterval.Min))
        {
            ggp::CCurve2dPtr pNewCurve = pWorldLine->Subdivide(oCutInterval.Min);
            std::swap(pNewCurve, pWorldLine);
            oCurves.push_back(pNewCurve);
            ggp::CPlane oCutPlane;
            oCutPlane.Create(ggp::CVector3d(pWorldLine->GetPoint(oCutInterval.Min), 0.0),
                ggp::CVector3d(pWorldLine->GetFirstDer(oCutInterval.Min), 0.0));
            ggp::CBody* pCutRes[2] = {0, 0};
            oBoolOpr.SplitBody(&oCutPlane, pBody.get(), pCutRes);
            pBody = pCutRes[0];
            if (pCutRes[1] && !pCutRes[1]->IsEmpty())
            {
                oObstacles.push_back(pCutRes[1]);
            }
        }
        if (oWorldLineRange.Contain(oCutInterval.Max))
        {
            ggp::CCurve2dPtr pNewCurve = pWorldLine->Subdivide(oCutInterval.Max);
            std::swap(pNewCurve, pWorldLine);
            ggp::CPlane oCutPlane;
            oCutPlane.Create(ggp::CVector3d(pWorldLine->GetPoint(oCutInterval.Max), 0.0),
                ggp::CVector3d(pWorldLine->GetFirstDer(oCutInterval.Max), 0.0));
            ggp::CBody* pCutRes[2] = {0, 0};
            oBoolOpr.SplitBody(&oCutPlane, pBody.get(), pCutRes);
            pBody = pCutRes[0];
            if (pCutRes[1] != nullptr)
            {
                pCutRes[1]->Free();
            }
        }
        else
        {
            pWorldLine = nullptr;
            pBody = nullptr;
        }
    }
    if (pWorldLine != nullptr)
    {
        oCurves.push_back(pWorldLine);
    }
    if (pBody && !pBody->IsEmpty())
    {
        oObstacles.push_back(pBody);
    }
}

void GTJRoutePlanCmdState::breakOtherObstacle(ggp::CCurve2dPtr pWorldLine,
    ggp::CBodyPtr pBody, 
    std::set<obstacleCutParam> oCutParams,
    QList<ggp::CBodyPtr>& oObstacles, 
    QList<ggp::CCurve2dPtr>& oCurves)
{
    QList<ggp::CCurve2dPtr> oValidCurves;
    ggp::CCurve2dPtr pOriginWorldLine = pWorldLine->Clone();
    for (auto iter = oCutParams.begin(); iter != oCutParams.end(); ++iter)
    {
        const obstacleCutParam& oParam = *iter;
        ggp::CIntervald oCutInterval(pWorldLine->GetNearestT(pOriginWorldLine->GetPoint(oParam.dT - oParam.dLength)), 
            pWorldLine->GetNearestT(pOriginWorldLine->GetPoint(oParam.dT + oParam.dLength)));
        ggp::CIntervald oWorldLineRange = pWorldLine->GetRange();
        if (oWorldLineRange.Contain(oCutInterval.Min))
        {
            ggp::CCurve2dPtr pCurve = pWorldLine->Subdivide(oCutInterval.Min);
            oValidCurves.push_back(pWorldLine);
            pWorldLine = pCurve;
        }
        if (oWorldLineRange.Contain(oCutInterval.Max))
        {
            pWorldLine = pOriginWorldLine->Subdivide(oCutInterval.Max);
        }
        else
        {
            pWorldLine = nullptr;
        }
    }
    if (pWorldLine != nullptr)
    {
        oValidCurves.push_back(pWorldLine);
    }

}

void GTJRoutePlanCmdState::exportAndFeedBack(ggp::CViewer* pViewer,
    QString& oFeedBackID,
    ggp::CVector3d& oPosition,
    ggp::CVector3d& oDir)
{
    auto vector3d2Json = [] (ggp::CVector3d& oVec) -> QJsonObject
    {
        QJsonObject res;
        res.insert(QString("x"), oVec.X);
        res.insert(QString("y"), oVec.Y);
        res.insert(QString("z"), oVec.Z);
        return res;
    };

    static QString s_cPathTemplate = "%1%2.3ds";
    static QString s_cImgPathTemplate = "%1%2.bmp";
    ggp::CGroupNode* pRootNode = pViewer->Scene()->GetRootNode();
    /*pViewer->ZoomAll();
    ggp::CVector3f oPos = pViewer->GetCamera()->Position();
    oPos.Z = 4e4;
    ggp::CVector3f oLookDir = pViewer->GetCamera()->Direction();
    ggp::CVector3f oUpAxis = pViewer->GetCamera()->UpDirection();
    double dHeight = fabs(oPos.Z);
    double dAngle = fabs(ggp::CVector3d(oPos.X, oPos.Y, 0.0).Angle(oLookDir.Vec3d()));
    double dLength = dHeight * tan(dAngle);
    ggp::CVector3d oBasePnt = ggp::CVector3d(oPos.X, oPos.Y, 0.0);
    ggp::CVector3d oDir = ggp::CVector3d(oLookDir.X, oLookDir.Y, 0.0);
    oDir.Normalize();
    oBasePnt = oBasePnt + oDir * dLength;
    oLookDir = oBasePnt.Vec3f();*/
    ggp::CScreenshoter sreenshot(pViewer);
    int nOrigH = pViewer->Height();
    int nOrigW = pViewer->Width();
    sreenshot.SetOffScreenBufferRange(nOrigW, nOrigH);
    CRefPtr<CImage> pModelViewImage = new CImage();
    sreenshot.CreateThumbnailScreenshot(pModelViewImage.get(), nOrigW, nOrigH, L""); 
    QString strImgPath = s_cImgPathTemplate.arg(QDir::tempPath())
        .arg(m_pCmdState->gmpService()->project()->projectName());
    ggp::CImageHelper::SaveImageToFile(pModelViewImage.get(), strImgPath.toStdWString());
    ggp::CGroupNode* pGroupNode = pViewer->Scene()->GetRootNode();
    G3dsExporter oExporter;
    QString strSavePath = s_cPathTemplate.arg(QDir::tempPath())
        .arg(m_pCmdState->gmpService()->project()->projectName());
    std::wstring path = strSavePath.toStdWString();
    oExporter.Export(pGroupNode, path);
    QJsonObject oJson;
    oJson.insert(QString("camerapos"), vector3d2Json(oPosition.Vec3d()));
    oJson.insert(QString("cameradir"), vector3d2Json(oDir.Vec3d()));
    oJson.insert(QString("cameraupdir"), vector3d2Json(oDir.Vec3d()));
    oJson.insert(QString("3dspath"), strSavePath);
    oJson.insert(QString("imgpath"), strImgPath);
    oJson.insert(QString("requestid"), oFeedBackID);
    QString aaa = QJsonDocument(oJson).toJson();
    //·¢Íù·þÎñÆ÷
    m_pCom->post(oJson);
}

QList<ggp::CBodyPtr> GTJRoutePlanCmdState::acquireFireObstale(int64_t nFloorID, QSet<int>& oSensorState)
{
    QList<ggp::CBodyPtr> oRes;
    ggp::CDBTable* pSensors = m_pCmdState->gmpService()->model()->projectDB()->GetTable(L"FireSensorMapping");
    IGMPElementDrawObjContnr* pEDOContainer = m_pCmdState->gmpService()->model()->edoContnr();
    if (pSensors)
    {
        ggp::CFileAddressList oSensorsOnThisFloor;
        pSensors->Search(pSensors->FindField(L"floorId"), nFloorID, &oSensorsOnThisFloor);
        for (int i = 0 ; i < oSensorsOnThisFloor.GetCount(); ++i)
        {
            FileAddress* pAddr = oSensorsOnThisFloor.GetItem(i);
            ggp::CDBRecord* pRec = pSensors->CreateRecordMap(*pAddr);
            ggp::CDBField* pField = pSensors->GetField(L"edoID");
            ggp::CDBField* pSensorID = pSensors->GetField(L"id");
            int64_t nEDOID = pField->GetInt64(pRec);
            int64_t nSensorID = pSensorID->GetInt64(pRec);
            IGMPElementDrawObj* pEDO = pEDOContainer->find(nEDOID);
            if (pEDO)
            {
                if (oSensorState.contains(nSensorID))
                {
                    pEDO->properties()->setAsInteger(pfnTransparent, c_nOnFireTransparent);
                    pEDO->state()->setValid(false);
                    pEDO->state()->setVisible(true);
                    IGMPSolidShape* pShape = dynamic_cast<IGMPSolidShape*>(pEDO->shape());
                    if (pShape)
                    {
                        oRes.push_back(pShape->body(false));
                    }
                }
                else
                {
                    pEDO->properties()->setAsInteger(pfnTransparent, c_nNotOnFireTransparent);
                    pEDO->state()->setVisible(false);
                    pEDO->state()->setValid(true);
                }
            }
            delete pRec;
        }
    }
    return oRes;
}

void GTJRoutePlanCmdState::initialize(QMap<IGMPElementDrawObj*, QList<IGMPElementDrawObj*>>& oExit,
    GTJRoutePlanStartEndPoint& oStartEndPoint,
    IGMPElementDrawObj* pStartEDO)
{
    acquireAllEmergencyExist(oExit, oStartEndPoint, pStartEDO);
    IGMPFloor* pCurFloor = m_pCmdState->gmpService()->state()->curFloor();
    double dFloorHeight = pCurFloor->height() * 1e3;
    double dBtmElev = pCurFloor->btmElev() * 1e3;
    ggp::CBox3d oFilterBox(ggp::CVector3d(-DBL_MAX,
        -DBL_MAX, dBtmElev + 500), ggp::CVector3d(DBL_MAX, DBL_MAX, dBtmElev + 2000));
    std::set<int> oObstacleElementType;
    oObstacleElementType.insert(etWall);
    oObstacleElementType.insert(gtj::etBrickWall);
    oObstacleElementType.insert(etInsulatingWall);
    oObstacleElementType.insert(etParapet);
    oObstacleElementType.insert(etRibbonWin);
    oObstacleElementType.insert(gtj::etCurtainWall);
    IGMPEdoIterator* pIter = createIterator(oFilterBox, &oObstacleElementType, false, g_DistEpsilon);
    if (m_pRoutePlaneModel)
    {
        delete m_pRoutePlaneModel;
    }
    m_pRoutePlaneModel = new GTJRouteplanModel;
    ggp::CBox2d oRangeBox;
    for (pIter->first(); !pIter->isDone(); pIter->next())
    {
        IGMPElementDrawObj* pCurItem = pIter->curItem();
        IGMPCustomLineSolidShape* pShape = dynamic_cast<IGMPCustomLineSolidShape*>(pCurItem->shape());
        if (pShape)
        {
            ggp::CPolygonPtr pWorldPoly = pShape->worldPoly(false);
            if (pWorldPoly)
            {
                ggp::CBox2d oPolyBox = pWorldPoly->Box();
                if (oRangeBox.NotEmpty())
                {
                    oRangeBox.MergeBox(oPolyBox);
                }
                else
                {
                    oRangeBox = oPolyBox;
                }
                QList<ggp::CBodyPtr> oBodies;
                QList<ggp::CCurve2dPtr> oCurves;
                breakObstacle(pShape, oBodies, oCurves, oExit);
                m_pRoutePlaneModel->m_obstacles.append(oBodies);
                m_pRoutePlaneModel->m_curves.append(oCurves);
                m_pRoutePlaneModel->m_polygonTree.insert(pWorldPoly->Clone(),
                    ggp::CBox3d(ggp::CVector3d(oPolyBox.MinPt(), 0.0),
                    ggp::CVector3d(oPolyBox.MaxPt(), 0.0)));
            }
        }
    }
    delete pIter;
    m_pRoutePlaneModel->m_rangeBox = oRangeBox;
    if (m_pQRCode != nullptr)
    {
        delete m_pQRCode;
    }
    m_pQRCode = new GTJQRCode;
}

void GTJRoutePlanCmdState::finalize()
{
    if (m_qrWidget)
    {
        m_qrWidget->close();
        m_qrWidget->destroy();
    }
    delete m_pRoutePlaneModel;
    delete m_pQRCode;
    delete m_pCom;
}

void GTJRoutePlanCmdState::onRequest(int64_t nPositionID, int64_t nFloorID, QString& oFeedBackID)
{
    QMap<IGMPElementDrawObj*, QList<IGMPElementDrawObj*>> oEmergencyExists;
    GTJRoutePlanStartEndPoint oStartEndPoint;
    IGMPElementDrawObj* pStartEDO = nullptr;
    if (!getPositionAndExit(nPositionID, 
        oStartEndPoint, 
        pStartEDO,
        oEmergencyExists))
    {
        return;
    }
    initialize(oEmergencyExists, 
        oStartEndPoint, 
        pStartEDO);
    QList<ggp::CBodyPtr> oStaticObstacle = m_pRoutePlaneModel->m_obstacles;
    SCOPE_EXIT{oStaticObstacle.swap(m_pRoutePlaneModel->m_obstacles);};
    QSet<int> oSensorState = std::move(parseSensorState(m_pCom->sensorState()));
    m_pRoutePlaneModel->m_obstacles.append(acquireFireObstale(nFloorID, oSensorState));
    double dFloorBtmElev = m_pCmdState->gmpService()->model()->regionContnr()->findFloorById(nFloorID)->btmElev() * 1e3;
    GTJRouteplan oPlaner(m_pRoutePlaneModel, 
        300, 
        dFloorBtmElev, 
        oStartEndPoint, 
        5);
    RRTConnectSolution* route = oPlaner.routeplan();
    if (route)
    {
        ggp::CViewer* pViewer = m_pCmdState->gmpService()->modelViewer()->RealViewer();
        GTJRouteRander rander(pViewer);
        rander.randerRoute(route, dFloorBtmElev);
        exportAndFeedBack(pViewer, oFeedBackID, route->position(), route->dir());
        if (route)
        {
            delete route;
        }
        QCoreApplication::processEvents();
    }
    return;
}

void GTJRoutePlanCmdState::onRoutePlanRequest(QString& strRequestParam)
{
    QStringList oParam = strRequestParam.split("@");
    if (oParam.size() == 3)
    {
        int64_t nFloorID = oParam[1].toInt();
        int64_t nPosID = oParam[2].toInt();
        onRequest(nPosID, nFloorID, strRequestParam);
    }
}

QSet<int> GTJRoutePlanCmdState::parseSensorState(const QString& strSensorStateInHex)
{
    bool ok;
    int nState = strSensorStateInHex.toInt(&ok, 16);
    QSet<int> oRes;
    int nIndex = 1;
    int nMask = 0x01;
    while (nState) {
        if (nState & nMask)
        {
            oRes.insert(nIndex);
        }
        nState &= ~nMask;
        nMask = nMask << 1;
        ++nIndex;
    }
    return oRes;
}

void GTJRoutePlanCmdState::showQR(int64_t nFloorID, int64_t nPosID,
    ggp::CVector2i& pos,
    ggp::CVector3d& oAnchor)
{
    m_qrWidget->showQR(QString("GlodonFantasticFour"), nFloorID, nPosID, oAnchor);
    m_qrWidget->updatePos(pos);
}

bool GTJRoutePlanCmdState::getPositionAndExit(int64_t nPositionID, 
    GTJRoutePlanStartEndPoint& oStartEndInfo,
    IGMPElementDrawObj*& pStartEDO,
    QMap<IGMPElementDrawObj*, QList<IGMPElementDrawObj*>>& oExit)
{
    bool bRes = false;
    IGMPElementDrawObjContnr* pContainer = m_pCmdState->gmpService()->model()->edoContnr();
    IGMPElementDrawObj* pDoor = pContainer->find(nPositionID);
    if (pDoor)
    {
        pStartEDO = pDoor;
        IGMPRelationOperator* pRelaOpr = m_pCmdState->gmpService()->model()->oprCenter()->relaOpr();
        IGMPElementDrawObj* pParent = pRelaOpr->parent(pDoor);
        addEmergencyExit(pParent, pDoor, oExit);
        oStartEndInfo.oStartPolygon = pDoor->shape()->worldPoly(false);
        ggp::CDBTable* pExits = m_pCmdState->gmpService()->model()->projectDB()->GetTable(L"EmergencyExit");
        if (pExits)
        {
            int64_t nFloorID = m_pCmdState->gmpService()->state()->curFloor()->iD();
            ggp::CFileAddressList oExitsOnThisFloor;
            pExits->Search(pExits->FindField(L"floorID"),
                nFloorID,
                &oExitsOnThisFloor);
            for (int i = 0; i< oExitsOnThisFloor.GetCount(); ++i)
            {
                ggp::CDBRecord* pRec = pExits->CreateRecordMap(*oExitsOnThisFloor.GetItem(i));
                ggp::CDBField* pField = pExits->GetField(L"range");
                oStartEndInfo.oExits.push_back(pField->GetPolygon(pRec)->Clone());
                delete pRec;
            }
            bRes = true;
        }
    }
    return bRes;
}

bool GTJRoutePlanCmdState::obstacleCutParam::operator<(const obstacleCutParam& other) const
{
    return dT < other.dT;
}

GTJQRCode::GTJQRCode()
    :m_code(nullptr)
{

}

GTJQRCode::~GTJQRCode()
{
    if (m_code != nullptr)
    {
        QRcode_free(m_code);
    }
}

void GTJQRCode::refresh(QString& sProjName, int64_t nFloorID, int64_t nPosID)
{
    static const QString s_Template = "%1@%2@%3";
    QString strQRContent = s_Template.arg(sProjName).arg(nFloorID).arg(nPosID);
    if(m_code != nullptr)
    {
        QRcode_free(m_code);
    }
    m_code = QRcode_encodeString(strQRContent.toStdString().c_str(),  
        1,
        QR_ECLEVEL_L,
        QR_MODE_8,
        1);
    update();
}

QSize GTJQRCode::sizeHint()
{
    return QSize(100, 100);
}

QSize GTJQRCode::minimumSize()
{
    return QSize(100, 100);
}

void GTJQRCode::paintEvent(QPaintEvent * e)
{
    QPainter painter(this);
    QColor background(Qt::white);
    painter.setBrush(background);
    painter.setPen(Qt::NoPen);
    painter.drawRect(0, 0, width(), height());
    if(m_code != NULL)
    {
        drawQR(painter, width(), height());
    }
}

void GTJQRCode::drawQR(QPainter& painter, int width, int height)
{
    QColor foreground(Qt::black);
    painter.setBrush(foreground);
    const int qr_width = m_code->width > 0 ? m_code->width : 1;
    double scale_x = width / qr_width;
    double scale_y = height / qr_width;
    for( int y = 0; y < qr_width; y ++)
    {  
        for(int x = 0; x < qr_width; x++)  
        {  
            unsigned char b = m_code->data[y * qr_width + x];
            if(b & 0x01)  
            {
                QRectF r(x * scale_x, y * scale_y, scale_x, scale_y);
                painter.drawRects(&r, 1);
            }
        }
    }
}

GTJQRWidget::GTJQRWidget(QWidget* parent /*= 0*/)
    :m_parent(parent)
{
    m_qrCode = new GTJQRCode();
    init();
}

GTJQRWidget::~GTJQRWidget()
{
    delete m_qrCode;
}

void GTJQRWidget::showQR(QString& sBuilding, 
    int64_t nFloorID, 
    int64_t nPosID,
    ggp::CVector3d& oAnchor)
{
    m_qrCode->refresh(sBuilding, nFloorID, nPosID);
    m_oAnchor = oAnchor;
    if (!this->isVisible())
    {
        this->show();
    }
}

void GTJQRWidget::destroy()
{
    this->close();
    this->deleteLater();
}

void GTJQRWidget::updatePos(ggp::CVector2i oPosOnParent)
{
    QSize oSize = this->size();
    QPoint oGlobalPos = m_parent->mapToGlobal(QPoint(oPosOnParent.X, oPosOnParent.Y));
    setGeometry(oGlobalPos.x(), oGlobalPos.y(), oSize.width(), oSize.height());
    this->raise();
    this->setFocus();
}

void GTJQRWidget::init()
{
    setWindowFlags (Qt::CustomizeWindowHint);
    setWindowFlags (Qt::FramelessWindowHint);
    this->setFixedSize(110, 110);
    QHBoxLayout* pLayout = new QHBoxLayout;
    m_qrCode->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    pLayout->addWidget(m_qrCode);
    this->setLayout(pLayout);
}

void GTJQRWidgetObserver::Update(unsigned int unEvent, void* pEventData)
{
    if (m_pControl && m_pViewer && m_pControl->isVisible())
    {
        int nScreenX, nScreenY;
        float dDummy;
        m_pViewer->WorldToScreen(m_pControl->m_oAnchor.Vec3f(), nScreenX, nScreenY, dDummy);
        m_pControl->updatePos(ggp::CVector2i(nScreenX, nScreenY));
    }
}

GTJQRWidgetObserver::GTJQRWidgetObserver(GTJQRWidget* pControl,
    ggp::CViewer* pViewer)
    :m_pControl(pControl), m_pViewer(pViewer)
{

}

GTJRoutePlanWebCom::GTJRoutePlanWebCom(GTJRoutePlanRequestCallBack& oCallBack)
{
    m_callBack = oCallBack;
    ICommunicate* pCom = getGAEAInterface();
    QJsonObject config;
    config.insert(QString("postURL"), QString("http://localhost:3000/center/phone_connect"));
    pCom->config(config);
    connect(pCom, SIGNAL(routePlanRequest(const QString&)), this, SLOT(onRequest(const QString&)));
}

GTJRoutePlanWebCom::~GTJRoutePlanWebCom()
{
    ICommunicate* pCom = getGAEAInterface();
    disconnect(pCom, SIGNAL(routePlanRequest(const QString&)), this, SLOT(onRequest(const QString&)));
}

void GTJRoutePlanWebCom::post(QJsonObject& data)
{
    getGAEAInterface()->post(data);
}

QString GTJRoutePlanWebCom::sensorState()
{
    return getGAEAInterface()->getSensorState();
}

void GTJRoutePlanWebCom::onRequest(const QString& strRequestParam)
{
    if (m_callBack)
    {
        m_callBack(strRequestParam);
    }
}