#include <numeric>
#include <memory>
#include <QString>

#include "GLDStrUtils.h"
#include "GEPEngineUtils.h"
#include "Common/scopeguard.h"
#include "Common/GTJElementFunction.h"
#include "Algorithm/alggeobuilder.h"
#include "Algorithm/algbodyextrema.h"
#include "Algorithm/algprojection.h"

#include "GMCommon/GMPCommon2d.h"
#include "GMCommon/GMPPositionFunc2d.h"
#include "GMCommon/GMPIntersectFunc2d.h"
#include "GMCommon/GMPOffSetFunc2d.h"
#include "GMPCore/GMPEditPolygonFrm.h"
#include "GMPCore/GMPSelParamPolyForm.h"
#include "GMPCore/GMPSystemOptions.h"
#include "GMPCore/GMPElementListModel.h"
#include "GMModel/GMPDBConsts.h"
#include "GMModel/GMPException.h"
#include "GMModel/IGMPModel.h"
#include "GMModel/IGMPElementDrawObj.h"
#include "GMModel/IGMPElement.h"
#include "GMModel/IGMPEdgeInfo.h"
#include "GMModel/IGMPShape.h"
#include "GMModel/IGMPShapeInfo.h"
#include "GMModel/IGMPElevOperator.h"
#include "GMModel/IGMPRelationOperator.h"
#include "GMModel/IGMPEObject.h"
#include "GMModel/IGMPDisplayOperator.h"

#include "GTJCommon/GTJCommon.h"
#include "GTJCommon/GTJPropCommon.h"
#include "GTJCommon/GTJSelParamPolyForm.h"
#include "GTJCommon/GTJCustomParamIntf.h"
#include "GTJCommon/GTJPolyColumnUtils.h"
#include "GTJCommon/GTJLineArchBodyConstruct.h"
#include "GTJCommon/ParamPoly/GTJNewParamTrench.h"
#include "GTJCommon/GCLCeilingPolyAdjust.h"
#include "GTJCommon/GTJPropInfoWriter.h"

#include "GTJDataService/DataService/GTJProjFolderGDBManager.h"
#include "GTJDataService/GTJRegionRuleManager.h"

#include "GCPPMCommon/ModelCommon/GCPPWallConnectionUtils.h"
#include "GTJQueryDrawingSetFrm.h"

#include "GMModel/../../src/GMModel/GMPNotifyOperator.h"
#include "GMModel/../../src/GMModel/GMPElement.h"
#include "GMPCore/GMPVectorDrawingFrm.h"
#include "VDECore/VDEVectorDrawingFrm.h"
#include "gtjsensorconfigwidget.h"

static const char* c_strClassName = "GTJComplexPropWriter";
static const char* c_strFloorBottomElev = QT_TRANSLATE_NOOP("GTJComplexPropWriter", "层底标高");

static const char* c_strLinetelHoleBottomElev = QT_TRANSLATE_NOOP("GGJLinetelBeamWriter", "洞口底标高");
static const char* c_strLinetelHoleTopElevAddBeamHeight = QT_TRANSLATE_NOOP("GGJLinetelBeamWriter", "洞口顶标高加过梁高度");
static const char* c_strLinetelAxisOffset = QT_TRANSLATE_NOOP("GGJLinetelBeamWriter", "过梁不能全部露在墙外,请输入 (%1,%2) 之间的实数!");
static const char *c_strInvalidNum = QT_TRANSLATE_NOOP("GGJLinetelBeamWriter", "输入非法,%1,请重新输入");
static const char *c_strLinetelPtLenInWall = QT_TRANSLATE_NOOP("GGJLinetelBeamWriter", "体自身相交,请重新输入");

static const char* c_pGTJPropInfoWriter = "GTJPropInfoWriter";
static const char* c_strSectionWriterType = QT_TRANSLATE_NOOP("GTJPropInfoWriter", "请输入 (0,50000] 之间的整数");
static const char* s_strTips = QT_TRANSLATE_NOOP("GTJPropInfoWriter", "提示");
static const char *c_strElevInvalid = QT_TRANSLATE_NOOP("GTJPropInfoWriter", "顶标高超出范围 [-1000,1000]m,请重新输入");
static const char *c_FatherDepthInvalid = QT_TRANSLATE_NOOP("GTJPropInfoWriter", "输入非法,请输入 (0,100000] 之间的整数,深度不允许小于冻土（湿土）厚度且父构件属性值不能大于 100000,请重新输入");
static const char* c_strParentElevInvalid = QT_TRANSLATE_NOOP("GTJPropInfoWriter", "父图元顶标高超出范围 [-1000,1000]m,请重新输入");
static const wchar_t   *c_szColumnCapType = L"ColumnCapType";
static const wchar_t   *c_szColumnBaseType = L"ColumnBaseType";
static const char*      c_szDitchEarthInvalidateElev = QT_TRANSLATE_NOOP("GTJPropInfoWriter", "图元底面斜面设置不对,使底面和顶面相交了,底面是不能高于顶面的");
static const char* c_strInvalidParamTrenchUnits = QT_TRANSLATE_NOOP("GTJPropInfoWriter", "底板,盖板,左右侧壁参数均为 0 时将删除已经绘制好的地沟图元，是否继续?");

static const char *c_szPropInfoWriter = "PropInfoWriter";
static const char *c_Confrim = QT_TRANSLATE_NOOP("PropInfoWriter", "确认");
static const char *c_ArchElevModifyHintInfo = QT_TRANSLATE_NOOP("PropInfoWriter", "修改标高后,如果标高和以前标高不同,会使拱梁变为平梁或斜梁,是否继续?");

static const char *const c_strBodyIntersected = QT_TRANSLATE_NOOP("GTJPropInfoWriter", "体自身相交,请重新输入");

static const char *const pEmbedBeamWidthHint = QT_TRANSLATE_NOOP("GTJPropInfoWriter", "请输入 (0,20000] 之间的整数");

static const wchar_t* pfnFrameLeftRightDeductSize	= L"FrameLeftRightDeductSize";                      //框左右扣尺寸
static const wchar_t* pfnFrameTopBottomDeductSize	= L"FrameTopBottomDeductSize";                      //框上下扣尺寸
static const wchar_t* pfnFrameArea					= L"FrameArea";										//框外圍面積

static const wchar_t* pfnDoorFrameLeftRightDeductSize					= L"DoorFrameLeftRightDeductSize";	 //門框左右扣尺寸
static const wchar_t* pfnDoorFrameTopBottomDeductSize					= L"DoorFrameTopBottomDeductSize";	 //門框上下扣尺寸

static const wchar_t* pfnWinFrameLeftRightDeductSize					= L"WinFrameLeftRightDeductSize";	 //窗框左右扣尺寸
static const wchar_t* pfnWinFrameTopBottomDeductSize					= L"WinFrameTopBottomDeductSize";	 //窗框上下扣尺寸

static const wchar_t* pfnVolume											= L"Volume";						 //體積
static const wchar_t* pfnDWWinWidth										= L"WinWidth";						 //門聯窗窗寬度

GTJParamNAbnormalWriter::GTJParamNAbnormalWriter(ggp::CDatabase* pParamDB, vector<int> * pAcceptEntTypes)
    : GMPParamNAbnormalWriter(pParamDB, pAcceptEntTypes), m_pParamDB(pParamDB)
{
}

void GTJParamNAbnormalWriter::onButtonClick(vector<GMPPropPtr>& oProps, bool& bModify)
{
    SCOPE_EXIT{ afterModifySectionInfo(oProps); };
    //具体处理逻辑
    if (oProps.empty())
    {
        return;
    }

    IGMPProperties* pProps = oProps[0]->owner();
    IGMPEObject* pTemp = pProps->owner();
    int nElementTypeID = -1;
    ggp::CDatabase* paramPolyDataBase = nullptr;
    if (IGMPElementDrawObj* pEDO = dynamic_cast<IGMPElementDrawObj*>(pTemp))
    {
        nElementTypeID = pEDO->elementType();
        paramPolyDataBase = pEDO->contnr()->model()->paramPolyDB();
        //因建模目前对于异形体属性没有做控制，导致升级一些构件会带截面形状属性，但里面没有Poly，会出现崩溃
        //此代码以后建模对于异形体属性控制处理完成之后可放开
        if (pEDO->element()->refType() == ertComplexBody)
        {
            return;
        }
    }
    else if (IGMPElement* pElement = dynamic_cast<IGMPElement*>(pTemp))
    {
        nElementTypeID = pElement->elementType();
        paramPolyDataBase = pElement->contnr()->model()->paramPolyDB();
        //因建模目前对于异形体属性没有做控制，导致升级一些构件会带截面形状属性，但里面没有Poly，会出现崩溃
        //此代码以后建模对于异形体属性控制处理完成之后可放开
        if (pElement->refType() == ertComplexBody)
        {
            return;
        }
    }

    if (-1 == nElementTypeID)
    {
        return;
    }

    int nSectionTypeID = oProps[0]->asInteger();
    if (esctSectAbnormity == nSectionTypeID)
    {
        editAbnormitySection(nElementTypeID, oProps, bModify);
    }
    else if (esctSectParams == nSectionTypeID)
    {
        const std::unique_ptr<GMPParamSectionInfo> pSectionInfo(new GMPParamSectionInfo());
        bModify = GTJSelParamPolyIntf::selCommonPoly(paramPolyDataBase, pTemp,
            pTemp->properties()->asString(pfnPolyValue), pTemp->properties()->asInteger(pfnPolyID),
            pSectionInfo.get(), new GTJWallParamWidget());
        if (bModify == false)
        {
            return;
        }
        vector<GMPPropPtr> tempProps = oProps;
        for (size_t j = 0; j < tempProps.size(); j++)
        {
            onWriteProp(tempProps[j]->owner(), pSectionInfo.get());
        }

        IGMPModel *pModel = nullptr;
        IGMPEObject* pChildObj = pProps->owner();
        IGMPEObject* pObj = getParentObject(pChildObj);

        CPolygonPtr pPolygon = str2Polygon(pSectionInfo->strSectionPoly);
        adjustPolygonCurve(pPolygon.get());

        if (pObj->type() == IGMPEObject::eoEDO)
        {
            IGMPElementDrawObj *pEDO = dynamic_cast<IGMPElementDrawObj*>(pObj);
            bool bIsBeam = pEDO->elementType() == etBeam;
            pModel = pEDO->contnr()->model();
            pModel->beginEdit();
            SCOPE_EXIT { pModel->endEdit(); };

            if (pPolygon && pPolygon->IsValid() && !pPolygon->IsEmpty())
            {
                //参数ID为381代表钢筋梯形梁，土建没有这个参数图
                //参数ID为371代表钢筋梯形次肋梁和土建梯形梁
                double dSectionWidth = 0.0;
                if (pSectionInfo->nPolyID == 381 || pSectionInfo->nPolyID == 371)
                {
                    QStringList strList = pSectionInfo->strPolyVal.split(';');
                    //这里限制为4个，再次确定它是梯形参数图
                    if (strList.count() == 4)
                        dSectionWidth = strList[1].toFloat();
                }
                if (ggp::sameValue(dSectionWidth, 0, ggp::g_DistEpsilon))
                    dSectionWidth = GetPolygonWidth(pPolygon.get());

                GMPPropPtr  pAxisOffsetProp = NULL;
                if (bIsBeam)
                {
                    pAxisOffsetProp = pProps->propByName(pfnAxisOffset_JZBZ);
                }
                else
                {
                    pAxisOffsetProp = pProps->propByName(pfnAxisOffset);
                }
                if (pAxisOffsetProp)
                {
                    double dAxisOffset = pAxisOffsetProp->asDouble();
                    if (ggp::compareValue(dAxisOffset, dSectionWidth, GMPEpsilonU) == vrGreaterThan)
                    {
                        pAxisOffsetProp->setIsNull();

                        // 如果是梁，则处理pfnAxisOffset_JZBZ
                        if (bIsBeam)
                        {
                            pEDO->properties()->setIsNull(pfnAxisOffset);
                        }
                    }
                }
            }
        }
        else if (pObj->type() == IGMPEObject::eoENT)
        {
            IGMPElement *const pElement = dynamic_cast<IGMPElement*>(pObj);
            if (pElement == nullptr)
            {
                return;
            }
            pElement->contnr()->model()->calculate();

            pModel = pElement->contnr()->model();
            pModel->beginEdit();
            SCOPE_EXIT { pModel->endEdit(); };
            double dSectionWidth = 0;
            if (!getProperWidthProperty(pProps, dSectionWidth))
            {
                return;
            }
            //参数化梯形梁特殊处理
            if (pPolygon && pPolygon->IsValid() && !pPolygon->IsEmpty())
            {
                //参数ID为381代表钢筋梯形梁，土建没有这个参数图
                //参数ID为371代表钢筋梯形次肋梁和土建梯形梁
                if (pSectionInfo->nPolyID == 381 || pSectionInfo->nPolyID == 371)
                {
                    QStringList strList = pSectionInfo->strPolyVal.split(';');
                    //这里限制为4个，再次确定它是梯形参数图
                    if (strList.count() == 4)
                        dSectionWidth = strList[1].toDouble();
                }
                if (ggp::sameValue(dSectionWidth, 0, ggp::g_DistEpsilon))
                {
                    dSectionWidth = GetPolygonWidth(pPolygon.get());
                }
            }

            if (pChildObj->elementType() == etTrenchUnit || pChildObj->elementType() == etStripFDUnit)
            {
                //地沟，条基比较特殊，做特殊处理
                changeEdoAxisOffset(pProps, pObj, pChildObj);
                return;
            }
            if (pProps->hasProp(pfnAxisOffset))
            {
                double dLeftWidth = pProps->asDouble(pfnAxisOffset);
                if (isGreater(dLeftWidth, dSectionWidth))
                {
                    pProps->setIsNull(pfnAxisOffset);
                }
                IGMPEdoIterator* iter = pElement->createEdoIter();
                iter->first();
                while (!iter->isDone())
                {
                    IGMPElementDrawObj* pEdo = iter->curItem();
                    IGMPCustomLineSolidShape* pShap = dynamic_cast<IGMPCustomLineSolidShape*>(pEdo->shape());
                    IGMPProperties* pEdoProps = pEdo->properties();
                    if (pShap)
                    {
                        double dLeftWidth = pShap->leftWidth();
                        if (isGreater(dLeftWidth, dSectionWidth))
                        {
                            pEdoProps->setIsNull(pfnAxisOffset);
                        }
                    }
                    iter->next();
                }
            }
            //对于梁和基础梁使用pfnAxisOffset_JZBZ
            GMPPropPtr  pAxisOffsetJZBZProp = pProps->propByName(pfnAxisOffset_JZBZ);
            if (pAxisOffsetJZBZProp)
            {
                double dAxisOffsaetJZBZ = pAxisOffsetJZBZProp->asDouble();
                if (ggp::compareValue(dAxisOffsaetJZBZ, dSectionWidth, GMPEpsilonU) == vrGreaterThan)
                {
                    pAxisOffsetJZBZProp->setIsNull();
                }
            }
        }
    }
}

/*!
*@brief  异形截面编辑
*@author liuxs-a 2015年1月9日
*/
void GTJParamNAbnormalWriter::editAbnormitySection(int nElementTypeID, std::vector<GMPPropPtr> &propVec, bool &modified)
{
    {
        std::vector<GMPPropPtr> tempProps = propVec;

        if (propVec.size() <= 0)
        {
            return;
        }

        IGMPProperties* pProps = propVec[0]->owner();
        IGMPEObject* pTemp = pProps->owner();

        int nElementTypeID = -1;

        if (IGMPElementDrawObj* pEDO = dynamic_cast<IGMPElementDrawObj*>(pTemp))
        {
            nElementTypeID = pEDO->elementType();
        }
        else if (IGMPElement* pElement = dynamic_cast<IGMPElement*>(pTemp))
        {
            nElementTypeID = pElement->elementType();
        }

        if (-1 == nElementTypeID)
        {
            return;
        }
        QWidget * activeWidget =  QApplication::activeWindow();
        //GlodonFrame 
        GMPVectorDrawingFrmEX oEditorParent(activeWidget, true, false);
        moveWidgetToScreenCenter(&oEditorParent, GMPSystemOptions::getInstance()->getMainWindow());
        SPolygonData sPolygonData;

        IGMPProperties * tempPropertites = propVec[0]->owner();
        if (tempPropertites)
        {
            GMPPropPtr SectionPolyProperty = tempPropertites->propByName(pfnSectionPoly);
            if (SectionPolyProperty)
            {
                CPolygonPtr polygonPtr = SectionPolyProperty->asPolygon();
                sPolygonData.pPolygon = polygonPtr->Clone();
            }
            GMPPropPtr insertPtProperty = tempPropertites->propByName(pfnInsertPt);
            if (insertPtProperty)
            {
                CVector2d vector2d = insertPtProperty->asVector2d();
                sPolygonData.oInsPoint = vector2d;
            }

            // 界面轴网信息，首先设置默认值
            sPolygonData.strHorzSpace = QString("100*10");
            sPolygonData.strVertSpace = QString("100*10");

            // 从属性解析轴网信息
            GMPPropPtr pSectionInfoProp = tempPropertites->propByName(L"SectionInfo");
            if (pSectionInfoProp != NULL)
            {
//                 CStreamPtr pStream = pSectionInfoProp->asStream();
//                 QByteArray data((char *)pStream->Buffer(), pStream->Length());
                QStringList sAxisInfo = pSectionInfoProp->asString().split(";");
                if (sAxisInfo.size() == 2)
                {
                    sPolygonData.strHorzSpace = sAxisInfo.at(0);
                    sPolygonData.strVertSpace = sAxisInfo.at(1);
                }
            }
            GMPPropPtr pVectorDrawingInfoProp = tempPropertites->propByName(L"VectorDrawingInfo");
            if (pVectorDrawingInfoProp != nullptr)
            {
                CStreamPtr pStream = pVectorDrawingInfoProp->asStream();

                //xiek 2017年6月14日fixed bug GTJY-17880 暂时截面编辑的构件类型，走poly显示，其他异形走流数据显示
                if (pStream != nullptr && 
                    nElementTypeID != etColumn &&
                    nElementTypeID !=etCustomLine &&
                    nElementTypeID != etTieColumn
                    )
                {
                    sPolygonData.oStream = QByteArray((char *)pStream->Buffer(), pStream->Length());
                }
            }
        }

        // gaoxu 修改数据前记录变量值，为了判断是否在对话框修改了数据
        CPolygon* pTmpPolygon = NULL;
        if (sPolygonData.pPolygon)
        {
            pTmpPolygon = sPolygonData.pPolygon->Clone();
        }
        CVector2d vPoint = sPolygonData.oInsPoint;
        QString tmpHorzSpace = sPolygonData.strHorzSpace;
        QString tmpVertSpace = sPolygonData.strVertSpace;

        oEditorParent.setPolygonData(sPolygonData);

        //xiek 2017年6月21日fixed bug GTJY-18046 设置插入点在修改后会自动调整
        oEditorParent.getEditPolygonFrm()->setInsertPointModify(true);
        if (oEditorParent.exec() == rtOk)
        {
            modified = false;
            //回写数据库
            if (tempPropertites)
            {
                oEditorParent.getPolygonData(sPolygonData);
                auto const nOffset = qRound(sPolygonData.pPolygon->Box().GetSize().X);
                IGMPModel *pModel = pProps->owner()->floor()->contnr()->model();
                pModel->beginEdit();
                for (size_t j=0; j < tempProps.size(); j++)
                {
                    IGMPProperties * jProperties = tempProps[j]->owner();
                    if (!sameVector(vPoint, sPolygonData.oInsPoint,g_DistEpsilon))
                    {
                        modified = true;
                    }
                    else
                    {
                        if ( (pTmpPolygon != NULL && sPolygonData.pPolygon == NULL)
                            ||(pTmpPolygon == NULL && sPolygonData.pPolygon != NULL))
                        {
                            modified = true;
                        }
                        else if(pTmpPolygon != NULL && sPolygonData.pPolygon != NULL)
                        {
                            CBooleanOperate2D oOperator;
                            CPolygonPtr pPolySub1 = oOperator.Subtract(pTmpPolygon, sPolygonData.pPolygon);
                            CPolygonPtr pPolySub2 = oOperator.Subtract(sPolygonData.pPolygon, pTmpPolygon);

                            if (pPolySub1 && pPolySub2 && !(pPolySub1->IsEmpty() && pPolySub2->IsEmpty()))
                            {
                                modified = true;
                            }
                        }
                    }                    

                    //jProperties->setAsPolygon(pfnSectionPoly, *sPolygonData.pPolygon->Clone());
                    jProperties->setAsVector2d(pfnInsertPt, sPolygonData.oInsPoint);

                    //Modified by yangwf：图元属性设置Poly时，是深拷贝；
                    setPoly(jProperties, sPolygonData.pPolygon);

                    //jiawc 2016年6月14日fixed bug 解决修改异形柱界面形状从圆弧到矩形崩溃的问题
                    GMPPropPtr pSectionWidthProp = jProperties->propByName(pfnSectionWidth);
                    if (NULL != pSectionWidthProp && !pSectionWidthProp->readOnly())
                    {
                        jProperties->setAsInteger(pfnSectionWidth, nOffset);
                    }

                    //lij-af 2015年7月30日fixed bug GMP-7580
                    // 当编辑异形属性而不是新建异形时，也设置界面轴网信息
                    GMPPropPtr pSectionInfoProp = jProperties->propByName(L"SectionInfo");
                    if (pSectionInfoProp != NULL)
                    {
                        GString sAxisGridInfo;
                        sAxisGridInfo.push_back(sPolygonData.strHorzSpace);
                        sAxisGridInfo.push_back(";");
                        sAxisGridInfo.push_back(sPolygonData.strVertSpace);
                        if (tmpHorzSpace != sPolygonData.strHorzSpace || tmpVertSpace != sPolygonData.strVertSpace)
                        {
                            modified = true;
                        }
                        QByteArray data = QVariant(sAxisGridInfo).toByteArray();
                        //CStream axisInfo(data.data(), data.size());
                        jProperties->setAsString(L"SectionInfo", sAxisGridInfo);
                    }

                    GMPPropPtr pVectorDrawingInfoProp = jProperties->propByName(L"VectorDrawingInfo");
                    if (pVectorDrawingInfoProp != NULL)
                    {
                        CStream vectorDrawingInfo(sPolygonData.oStream, sPolygonData.oStream.size());
                        jProperties->setAsStream(L"VectorDrawingInfo", vectorDrawingInfo);
                    }
                    //end
                }
                pModel->endEdit();

                for (auto itr = tempProps.cbegin(), itrEnd = tempProps.cend(); itr != itrEnd; ++itr)
                {
                    updateAxisOffset((*itr)->owner(), nOffset);
                }
            }
        }
        if (pTmpPolygon)
        {
            pTmpPolygon->Free();
        }
    }
    return;
    // 在堆上创建，fixing GTJ-1653
    std::unique_ptr<GMPEditPolygonFrmEX> pPolygonFrm(new GMPEditPolygonFrmEX(QApplication::activeWindow()));
    switch (nElementTypeID)
    {
        case gtj::etForceWall: case gtj::etBrickWall: case etCustomLine:
        case etBeam: case etLinkBeam: case etTieBeam: case etFDBeam:
        case etStripFDUnit: case etReportCTL: case etEave: case etParapet: case etCoping: case etMainRibsBeam:
        {
            pPolygonFrm->getEditPolygonFrm()->editInsertPoint(false);
            break;
        }
        case etBedding:
        {
            if (!propVec.empty() && propVec.front()->schema()->entSubTypeID() == 224)
            { // 线式异形垫层
                pPolygonFrm->getEditPolygonFrm()->editInsertPoint(false);
            }
            break;
        }
		default:
			break;
    }

    SPolygonData polygonData;
    IGMPProperties *pPropVec = nullptr;
    if (!propVec.empty())
    {
        pPropVec = propVec.front()->owner();
    }
    if (pPropVec != nullptr)
    {
        GMPPropPtr pPropTemp = pPropVec->propByName(pfnSectionPoly);
        if (pPropTemp != nullptr)
        {
            polygonData.pPolygon = pPropTemp->asPolygon()->Clone();
        }
        pPropTemp = pPropVec->propByName(pfnInsertPt);
        if (pPropTemp != nullptr)
        {
            polygonData.oInsPoint = pPropTemp->asVector2d();
        }
        // 从属性解析轴网信息
        pPropTemp = pPropVec->propByName(L"SectionInfo");
        if (pPropTemp != nullptr)
        {
            auto const axisInfo = pPropTemp->asString();
            if (!axisInfo.isEmpty())
            {
                polygonData.strHorzSpace = axisInfo.section(';', 0, 0, QString::SectionSkipEmpty);
                polygonData.strVertSpace = axisInfo.section(';', -1, -1, QString::SectionSkipEmpty);
            }
        }
        else
        {
            polygonData.strHorzSpace = polygonData.strVertSpace = "100*10";
        }
    }
    pPolygonFrm->getEditPolygonFrm()->setPolygonData(polygonData);
    if (pPolygonFrm->exec() == rtOk)
    { //回写数据库
        if (pPropVec != nullptr)
        {
            IGMPModel *pModel = pPropVec->owner()->floor()->contnr()->model();
            pModel->beginEdit();
            pPolygonFrm->getEditPolygonFrm()->getPolygonData(polygonData);
            const QString axisInfo = QString("%1;%2").arg(polygonData.strHorzSpace).arg(polygonData.strVertSpace);
            auto const nWidth = qRound(polygonData.pPolygon->Box().GetSize().X);
            std::set<GMPPropPtr> setModifiedProps;
            for (auto itr = propVec.cbegin(), itrEnd = propVec.cend(); itr != itrEnd; ++itr)
            {
                //zhangh-t fixed bug 
                if (setModifiedProps.find((*itr)) != setModifiedProps.end())
                {
                    continue;
                }
                setModifiedProps.insert((*itr));
                pPropVec = (*itr)->owner();
                //Modified by yangwf：图元属性设置Poly时，是深拷贝；
                setPoly(pPropVec, polygonData.pPolygon);
                
                //if (pPropVec->hasProp(pfnSectionWidth))
                //jiawc 2016年6月14日fixed bug 解决修改异形柱界面形状从圆弧到矩形崩溃的问题
                GMPPropPtr pSectionWidthProp = pPropVec->propByName(pfnSectionWidth);
                if (NULL != pSectionWidthProp && !pSectionWidthProp->readOnly())
                {
                    pPropVec->setAsInteger(pfnSectionWidth, nWidth);
                }
                pPropVec->setAsVector2d(pfnInsertPt, polygonData.oInsPoint);
                pPropVec->setAsString(L"SectionInfo", axisInfo);
                modified = true;
            }
            pModel->endEdit();
            for (auto itr = propVec.cbegin(), itrEnd = propVec.cend(); itr != itrEnd; ++itr)
            {
                 updateAxisOffset(pPropVec, nWidth);
            }
        }
    }
}

void GTJParamNAbnormalWriter::setPoly(IGMPProperties * pProperties, CPolygon * pPoly)
{
    //清空边坡信息
    this->clearEdgeInfos(pProperties->owner());
    pProperties->setAsPolygon(pfnSectionPoly, *pPoly);
}
/*!
*@brief  清空边坡信息
*@author chengdz 2015年5月8日
*/
void GTJParamNAbnormalWriter::clearEdgeInfos(IGMPEObject* pObj)
{
    //清空图元的边坡信息
    if (pObj->type() == IGMPEObject::eoEDO)
    {
        IGMPElementDrawObj *pEDO = dynamic_cast<IGMPElementDrawObj*>(pObj);
        //清空图元的边坡信息
        IGMPPointSolidShape* pPointShape = dynamic_cast<IGMPPointSolidShape*>(pEDO->shape());
        if (pPointShape != nullptr)
        {
            pPointShape->getEdgeInfos()->clear();
        }
    }
}

void GTJParamNAbnormalWriter::updateAxisOffset(IGMPProperties *const pPropList, const double dWidth)
{
    IGMPEObject *const pChildObject = pPropList->owner();
    IGMPEObject *const pObject = getParentObject(pChildObject);
    if (pObject->type() == IGMPEObject::eoEDO)
    { // 当前图元
        IGMPElementDrawObj *const pEdo = dynamic_cast<IGMPElementDrawObj *>(pObject);
        IGMPCustomLineSolidShape *const pShape = dynamic_cast<IGMPCustomLineSolidShape *>(pEdo->shape());
        if (pShape != nullptr)
        {
            const bool mirror = pShape->isMirror();
            if (pPropList->hasProp(pfnAxisOffset))
            {
                double dLeftWidth = pShape->realLeftWidth(false);
                if (ggp::IsGreaterThan(dLeftWidth, dWidth, ggp::g_DistEpsilon))
                {
                    pPropList->setIsNull(pfnAxisOffset);
                    if (pPropList->hasProp(L"UserChangeAxis"))
                    {
                        pPropList->setAsBoolean(L"UserChangeAxis", false);;
                    }
                }
                else if (ggp::IsLessEqualThan(dLeftWidth, 0, ggp::g_DistEpsilon))
                {
                    pPropList->setAsDouble(pfnAxisOffset, mirror ? dWidth : 0);
                }
            }
            /*pEdo->contnr()->model()->calculate();*/
            if (pPropList->hasProp(pfnAxisOffset_JZBZ))
            { // 求真正的左宽度 add by shenc
                double dTemp;
                if (!pPropList->hasProp(L"UserChangeAxis") || !pPropList->asBoolean(L"UserChangeAxis"))
                {
                    dTemp = pPropList->asDouble(pfnAxisOffset_JZBZ);
                    const double dOffset = (mirror ? dWidth - dTemp : dTemp);
                    if (ggp::IsGreaterThan(dOffset, dWidth, ggp::g_DistEpsilon))
                    {
                        pPropList->setIsNull(pfnAxisOffset_JZBZ);
                    }
                    if (ggp::IsGreaterThan(0, dOffset, ggp::g_DistEpsilon))
                    {
                        pPropList->setAsDouble(pfnAxisOffset_JZBZ, mirror ? dWidth : 0);
                    }
                }
//                 if (pPropList->hasProp(L"UserChangeAxis") && pPropList->asBoolean(L"UserChangeAxis"))
//                 {
// //                     dTemp = pPropList->asDouble(pfnAxisOffset);
// //                     if (ggp::IsGreaterThan(dTemp, dWidth, ggp::g_DistEpsilon))
// //                     {
// //                         pPropList->setIsNull(pfnAxisOffset);
// //                     }
// //                     else
// //                     {
// //                         pPropList->setAsDouble(pfnAxisOffset, dTemp);
// //                     }
//                 }
//                 else
//                 {
//                     dTemp = pPropList->asDouble(pfnAxisOffset_JZBZ);
//                     const double dOffset = (mirror ? dWidth - dTemp : dTemp);
//                     if (ggp::IsGreaterThan(dOffset, dWidth, ggp::g_DistEpsilon))
//                     {
//                         pPropList->setIsNull(pfnAxisOffset_JZBZ);
//                     }
//                     if (ggp::IsGreaterThan(0, dOffset, ggp::g_DistEpsilon))
//                     {
//                         pPropList->setAsDouble(pfnAxisOffset_JZBZ, mirror ? dWidth : 0);
//                     }
//                 }
            }
        }
    }
    else if (pObject->type() == IGMPEObject::eoENT)
    {
        auto const elmType = pChildObject->elementType();
        if (elmType == etTrenchUnit || elmType == etStripFDUnit)
        {
            //地沟，条基比较特殊，做特殊处理
            changeEdoAxisOffset(pPropList, pObject, pChildObject, dWidth);
            return;
        }
        if (pPropList->hasProp(pfnAxisOffset))
        {
            double dOffset = pPropList->asDouble(pfnAxisOffset);
            if (ggp::IsGreaterThan(dOffset, dWidth, ggp::g_DistEpsilon))
            {
                pPropList->setIsNull(pfnAxisOffset);
            }
            const std::unique_ptr<IGMPEdoIterator> iterator(dynamic_cast<IGMPElement *>(pObject)->createEdoIter());
            IGMPElementDrawObj *pEdo;
            for (iterator->first(); !iterator->isDone(); iterator->next())
            {
                pEdo = iterator->curItem();
                IGMPCustomLineSolidShape *pShape = dynamic_cast<IGMPCustomLineSolidShape*>(pEdo->shape());
                if (pShape != nullptr)
                {
                    if (ggp::IsGreaterThan(pShape->leftWidth(), dWidth, ggp::g_DistEpsilon))
                    {
                        pEdo->properties()->setIsNull(pfnAxisOffset);
                    }
                }
            }
        }
        if (pPropList->hasProp(pfnAxisOffset_JZBZ))
        { // 对于梁和基础梁使用pfnAxisOffset_JZBZ
            const double dOffset = pPropList->asDouble(pfnAxisOffset_JZBZ);
            if (ggp::IsGreaterThan(dOffset, dWidth, GMPEpsilonU))
            {
                pPropList->setIsNull(pfnAxisOffset_JZBZ);
            }
        }
    }
}

/*!
*@brief      重写父类函数 fix-TJGJ-27732
*@author     qinyb-a 2015年8月22日
*@parameter: IGMPProperties * pProperties
*@parameter: const GMPParamSectionInfo * pSectionInfo
*@return     void
*/
void GTJParamNAbnormalWriter::onWriteProp(IGMPProperties* pProperties, const GMPParamSectionInfo * pSectionInfo)
{
    IGMPModel *pModel = nullptr;
    IGMPEObject *pObj = pProperties->owner();
    if (pObj->type() == IGMPEObject::eoENT)
    {
        IGMPElement *pElement = dynamic_cast<IGMPElement*>(pObj);
        pModel = pElement->contnr()->model();
    }
    else
    {
        IGMPElementDrawObj *pEDO = dynamic_cast<IGMPElementDrawObj*>(pObj);
        pModel = pEDO->contnr()->model();
    }
    pModel->beginEdit();

    //判断当前构件是否为参数化柱或构造柱
    bool bIsParam = false;
    if (pObj->elementType() == etColumn || pObj->elementType() == etTieColumn)
    {
        const int paramColumn = 3;
        const int paramTieColumn = 127;
        if (pObj->elementSubType() == paramColumn || pObj->elementSubType() == paramTieColumn)
        {
            bIsParam = true;
        }
    }
    if (bIsParam) //只有构造柱、柱参数化时才写入
    {
        GString strsectioninf = paramPolyToString(pSectionInfo->strSectionPoly,
            pSectionInfo->nPolyID, pSectionInfo->strPolyVal, pSectionInfo->strInsPt);
        pProperties->setAsString(GTJDBCConsts::pfnSectionInfo, strsectioninf);
    }

    pProperties->setAsInteger(pfnPolyID, pSectionInfo->nPolyID);
    pProperties->setAsString(pfnPolyValue, pSectionInfo->strPolyVal);// fixed bug TJGJ-2177 by zl
    CPolygon* pPolygon = str2Polygon(pSectionInfo->strSectionPoly);
    if (pPolygon)
    {
        adjustPolygon(pPolygon);
        pProperties->setAsPolygon(pfnSectionPoly, *pPolygon);
        pPolygon->Free();
    }

    // 设置插入点
    QStringList oInsertPt = pSectionInfo->strInsPt.split(L',');
    assert(oInsertPt.count() == 2);
    pProperties->setAsVector2d(pfnInsertPt, CVector2d(oInsertPt[0].toDouble(), oInsertPt[1].toDouble()));

    //pProperties->setAsString(pfnPolyName, pSectionInfo->strPolyName);
    pModel->endEdit();
}

bool GTJParamNAbnormalWriter::getProperWidthProperty(IGMPProperties *const pPropVec, double &dWidth)
{
    if (pPropVec->hasProp(pfnThickness))
    {
        dWidth = pPropVec->asDouble(pfnThickness);
        return true;
    }
    else if (pPropVec->hasProp(pfnSectionWidth))
    {
        dWidth = pPropVec->asDouble(pfnSectionWidth);
        return true;
    }
    else if (pPropVec->hasProp(pfnSectionRadius))
    {
        dWidth = pPropVec->asDouble(pfnSectionRadius);
        return true;
    }
    return false;
}

IGMPEObject* GTJParamNAbnormalWriter::getParentObject(IGMPEObject *const pObject)
{
    if (pObject != nullptr)
    {
        auto const elmType = pObject->elementType();
        if (elmType == etStripFDUnit || elmType == etTrenchUnit)
        {
            IGMPRelationOperator *const pRelaOpr = pObject->floor()->contnr()->model()->oprCenter()->relaOpr();
            if (pObject->type() == IGMPEObject::eoENT)
            {
                return pRelaOpr->parent(dynamic_cast<IGMPElement*>(pObject));
            }
            else
            {
                return pRelaOpr->parent(dynamic_cast<IGMPElementDrawObj *>(pObject));
            }
        }
    }
    return pObject;
}

void GTJParamNAbnormalWriter::changeEdoAxisOffset(IGMPProperties* pProps, IGMPEObject * pObj, IGMPEObject* pChildObj, const double dWidth/*= -1*/)
{
    //修改构建下所有图元属性
    IGMPElement *pParentElement = dynamic_cast<IGMPElement*>(pObj);
    if (pParentElement == nullptr)
    {
        return;
    }
    if (pParentElement->properties()->hasProp(pfnAxisOffset))
    {
        double dLeftWidth = pParentElement->properties()->asDouble(pfnAxisOffset);
        // fix bug GTJY-5765
        double dLineWidth = dWidth;
        if (ggp::IsLessThan(dLineWidth, 0, g_DistEpsilon))
            dLineWidth = pChildObj->properties()->asDouble(pfnSectionWidth);
        if (ggp::IsGreaterThan(dLeftWidth, dLineWidth, getDefaultDistEpsilon()))
        {
            pParentElement->properties()->setIsNull(pfnAxisOffset);
        }
        IGMPEdoIterator *pEdoContnr = pParentElement->createEdoIter(true);
        pEdoContnr->first();
        while (!pEdoContnr->isDone())
        {
            IGMPElementDrawObj *pEDO = pEdoContnr->curItem();
            pEDO->properties()->setIsNull(pfnAxisOffset);
            pEdoContnr->next();
        }
    }
}

void GTJParamNAbnormalWriter::afterModifySectionInfo(vector<GMPPropPtr>& oProps)
{
    m_setModifiedElements.clear();
}

bool GTJParamNAbnormalWriter::isModified(IGMPEObject* pObj)
{
    bool bRet = false;
    if (pObj != nullptr)
    {
        IGMPElement* pElement = nullptr;
        if (pObj->type() == IGMPEObject::eoEDO)
        {
            pElement = dynamic_cast<IGMPElementDrawObj*>(pObj)->element();
        }
        else
        {
            pElement = dynamic_cast<IGMPElement*>(pObj);
        }
        bRet = m_setModifiedElements.find(pElement) != m_setModifiedElements.end();
    }
    return bRet;
}

GTJComplexDisplayStylePropWriter::GTJComplexDisplayStylePropWriter(vector<int> * pAcceptEntTypes /*= nullptr*/)
    :GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.push_back(GString::fromStdWString(pfnBorderColor));
    m_AcceptProps.push_back(GString::fromStdWString(pfnFillColor));
    m_AcceptProps.push_back(GString::fromStdWString(pfnTransparent));
}

GTJComplexDisplayStylePropWriter ::~GTJComplexDisplayStylePropWriter()
{
}

void GTJComplexDisplayStylePropWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    Q_ASSERT(pProp);
    Q_ASSERT(pProp->owner());
    Q_ASSERT(pProp->owner()->owner());

    GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
    const std::wstring& sName = pProp->propName();
    IGMPEObject* pObj = pProp->owner()->owner();
    if (pObj->type() == IGMPEObject::eoENT)
    {
        IGMPElement* pElement = dynamic_cast<IGMPElement*>(pObj);
        IGMPRelationOperator* pRelaOpr = pElement->contnr()->model()->oprCenter()->relaOpr();
        const int nCount = pRelaOpr->childrenCount(pElement);
        for (int i = 0; i < nCount; i++)
        {
            IGMPElement *pChild = pRelaOpr->child(pElement, i);
            if (strNewValue.isEmpty())
            {
                pChild->properties()->setIsNull(sName);
            }
            else/* if (ggp::sameValue(nOldPropValue, pChild->properties()->asInteger(sName), getDefaultDistEpsilon()))*/
            {
                pChild->properties()->setAsInteger(sName, strNewValue.toInt());
            }
        }
    }
    if (pObj->type() == IGMPEObject::eoEDO)
    {
        IGMPElementDrawObj* pEDO = dynamic_cast<IGMPElementDrawObj*>(pObj);
        IGMPRelationOperator* pRelaOpr = pEDO->contnr()->model()->oprCenter()->relaOpr();
        const int nCount = pRelaOpr->childrenCount(pEDO);
        for (int i = 0; i < nCount; i++)
        {
            IGMPElementDrawObj *pChild = pRelaOpr->child(pEDO, i);
            if (strNewValue.isEmpty())
            {
                pChild->properties()->setIsNull(sName);
            }
            else/* if (ggp::sameValue(nOldPropValue, pChild->properties()->asInteger(sName), getDefaultDistEpsilon()))*/
            {
                pChild->properties()->setAsInteger(sName, strNewValue.toInt());
            }
        }
    }
}

GTJComplexPropWriter::GTJComplexPropWriter(vector<int> * pAcceptEntTypes /*= nullptr*/)
    :GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.push_back(GString::fromStdWString(pfnTopElev));
    m_AcceptProps.push_back(GString::fromStdWString(pfnBottomElev));
    m_AcceptProps.push_back(GString::fromStdWString(pfnHeight));
}

GTJComplexPropWriter::~GTJComplexPropWriter()
{
}

void GTJComplexPropWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    //原则 始终保持高度不变
    //如果修改顶标高 则同时修改低标高
    //如果修改底标高 则同时修改顶标高
    //如果修改高度 则同时修改顶标高
    Q_ASSERT(pProp);
    Q_ASSERT(pProp->owner());
    Q_ASSERT(pProp->owner()->hasProp(pfnHeight) || pProp->owner()->hasProp(pfnDepth));

    auto safeSetProp = [&] (GMPPropPtr prop, const GString&sValue, bool bCheck) {
        GString strValue = GTJPropCommon::convertElevStr2(sValue);
        GString strTransfer = GTJPropCommon::unitTransferElev(prop, strValue, m_pUnitTransfer);
        // 根据数据类型设置值，标高目前只会是gmdtText，因为数据库中对应的是Memo
        prop->setAsString(strTransfer, bCheck);
    };

    //先做默认值和空值的处理
    // 恢复默认值
    const std::wstring&sName = pProp->propName();
    if (strNewValue.trimmed().isEmpty())
    {
        if (sName == pfnTopElev)
        {
            double dHeight = pProp->owner()->asDouble(pfnHeight) / 1000.0;
            safeSetProp(pProp, qApp->translate(c_strClassName, c_strFloorBottomElev) + "+" + QString::number(dHeight), true);
            pProp->owner()->propByName(pfnBottomElev)->initDefaultValue();
        }
        else if (sName == pfnBottomElev)
        {
            double dHeight = pProp->owner()->asDouble(pfnHeight) / 1000.0;
            GMPPropPtr pTopElev = pProp->owner()->propByName(pfnTopElev);
            safeSetProp(pTopElev, qApp->translate(c_strClassName, c_strFloorBottomElev) +
                "+" + QString::number(dHeight), false);
            pProp->initDefaultValue();
        }
        else if (sName == pfnHeight)//目前父构件的高度是只读的 这里没什么用 暂时留着吧
        {
            pProp->initDefaultValue();
            double dHeight = pProp->owner()->asDouble(pfnHeight) / 1000.0;
            GMPPropPtr pTopElev = pProp->owner()->propByName(pfnTopElev);
            safeSetProp(pTopElev, qApp->translate(c_strClassName, c_strFloorBottomElev) +
                "+" + QString::number(dHeight), true);
        }
        return;
    }


    //这里做了假设认为高度的单位是mm而标高的单位是m
    __int64 nElementType = pProp->owner()->owner()->elementType();
    if (sName == pfnTopElev)
    {
        double dHeight = 0.0;
        if (nElementType == etExcavateEarthBackfill)
        {
            dHeight = pProp->owner()->asDouble(pfnDepth) / 1000.0;
        }
        else
        {
            dHeight = pProp->owner()->asDouble(pfnHeight) / 1000.0;
        }

        if (::compareValue(dHeight, 0.0, GMPEpsilonU) != gvrEqualsValue)
        {
            safeSetProp(pProp->owner()->propByName(pfnBottomElev), strNewValue + "-" + QString::number(dHeight), false);
        }
    }
    else if (sName == pfnBottomElev)
    {
        double dHeight = 0.0;
        if (nElementType == etExcavateEarthBackfill)
        {
            dHeight = pProp->owner()->asDouble(pfnDepth) / 1000.0;
        }
        else
        {
            dHeight = pProp->owner()->asDouble(pfnHeight) / 1000.0;
        }
        if (::compareValue(dHeight, 0.0, GMPEpsilonU) != gvrEqualsValue)
        {
            safeSetProp(pProp->owner()->propByName(pfnTopElev), strNewValue + "+" + QString::number(dHeight), false);
        }
    }
    else if (sName == pfnHeight)//目前父构件的高度是只读的 这里没什么用 暂时留着吧
    {
        GString sBottomElev = pProp->owner()->asString(pfnBottomElev);
        safeSetProp(pProp->owner()->propByName(pfnTopElev), sBottomElev +
            "+" + QString::number(strNewValue.toDouble() / 1000.0), true);
    }
    //输入表达式不做处理 FIX BUG TJGJ-28846
    GString strTransfer = GTJPropCommon::unitTransferElev(pProp, strNewValue, m_pUnitTransfer);
    // 根据数据类型设置值，标高目前只会是gmdtText，因为数据库中对应的是Memo
    pProp->setAsString(strTransfer, true);
}

GTJComplexHeightListener::GTJComplexHeightListener(IGMPService *pIGMPService)
{
    m_pIGMPService = pIGMPService;
}

void GTJComplexHeightListener::onBeginEdit()
{
    m_oMap.clear();
}

void GTJComplexHeightListener::onEndEdit()
{
    m_pIGMPService->model()->beginEdit(false);
    SCOPE_EXIT{m_pIGMPService->model()->endEdit();};

    for (auto iter = m_oMap.begin(); iter != m_oMap.end(); ++iter)
    {
        QP_FUN("GTJComplexHeightListener::onEndEdit()")
        IGMPProperties *pProps = iter->first;
        IGMPEObject *pOwner = pProps->owner();
        IGMPPropertySchema *pSchema = iter->second;

        GString sBottomElev;
        //这里做了假设认为高度的单位是mm而标高的单位是m
        double dHeight = pProps->asDouble(pfnHeight) / 1000.0;

        if (pOwner->type() == IGMPEObject::eoENT && pSchema->publicFlag() == true &&
            ::compareValue(dHeight, 0.0, GMPEpsilonU) != gvrEqualsValue)
        {
            IGMPElement *pElement = dynamic_cast <IGMPElement*>(pOwner);

            IGMPEdoIterator *pEdo = pElement->createEdoIter();
            pEdo->first();
            while (!pEdo->isDone())
            {
                IGMPProperties *pProperTies = pEdo->curItem()->properties();
                sBottomElev = pProperTies->asString(pfnBottomElev);
                QString strElevValue = GTJPropCommon::convertElevStr(sBottomElev + "+" + QString::number(dHeight));
                pProperTies->setAsString(pfnTopElev, strElevValue, false);
                pEdo->next();
            }
            delete pEdo;
            pEdo = nullptr;
        }
        if (pProps->isDataNull(pfnBottomElev))
        {
            continue;
        }
        sBottomElev = pProps->asString(pfnBottomElev);
        if (::compareValue(dHeight, 0.0, GMPEpsilonU) != gvrEqualsValue)
        {
            if (isNumeric(sBottomElev))
            {
                double dBottomElev = sBottomElev.toDouble();
                double dTopElev = dBottomElev + dHeight;
                GString strValue = QString::number(dTopElev, 'f', 3);
                formatStrNum(strValue);

                pProps->setAsString(pfnTopElev, strValue, false);
            }
            else
            {
                QString strElevValue = GTJPropCommon::convertElevStr(sBottomElev + "+" + QString::number(dHeight));
                pProps->setAsString(pfnTopElev, strElevValue, false);
            }
        }
        else
        {
            pProps->setAsString(pfnTopElev, sBottomElev);
        }
    }
    m_oMap.clear();
}

void GTJComplexHeightListener::onNotify(int iNotifyType, IGMPProperties* pProps, void* pData, bool bRedoUndo)
{
    auto const elmTypeId = pProps->owner()->elementType();
    if (!bRedoUndo && iNotifyType == ntAfterUpdate && (elmTypeId == etPileCap || elmTypeId == etIsolatedFD))
    {
        IGMPPropertySchema *pSchema = reinterpret_cast<IGMPPropertySchema*>(pData);
        if ((pSchema->propName() == pfnHeight))
        {
            m_oMap[pProps] = pSchema;
        }
    }
}
//构件
void GTJComplexHeightListener::onNotify(int iNotifyType, IGMPElement* pEnt, void* pData, bool bRedoUndo)
{
    if (iNotifyType == ntBeforeDelete)
    {
        auto iter = m_oMap.find(pEnt->properties());
        if (iter != m_oMap.end())
        {
            m_oMap.erase(iter);
        }
    }
    if (iNotifyType != ntAfterUpdate)
        return;
    //处理名称与类别联动
//     auto const nElementType = pEnt->elementType();
//     if (nElementType != etBeam && nElementType != etFDBeam && nElementType != etSideBeam
//         && nElementType != etMainRibsBeam && nElementType != etColumn && nElementType != etTieColumn)
//     {
//         return;
//     }
//     GMPPropPtr const pIGMPProperty = pEnt->properties()->propByName(pfnDescription);
//     if (pIGMPProperty != nullptr)
//     {
//         m_pIGMPService->propInfoWriters()->writeProp(pIGMPProperty, pEnt->name(), "0");
//     }
}

//图元
void GTJComplexHeightListener::onNotify(int iNotifyType, IGMPElementDrawObj* pEdo, void* pData, bool bRedoUndo)
{
    if (iNotifyType == ntBeforeDelete || iNotifyType == ntReleaseEdo)
    {
        auto iter = m_oMap.find(pEdo->properties());
        if (iter != m_oMap.end())
        {
            m_oMap.erase(iter);
        }
    }
}

//////////////////////////////////////////////////////////////////////////
GTJSpecialParamTypeWriter::GTJSpecialParamTypeWriter(ggp::CDatabase* pParamDB, vector<int> * pAcceptEntTypes)
    : GMPPropInfoWriter(pAcceptEntTypes), m_pParamDB(pParamDB)
{
    m_AcceptProps.push_back(GString::fromWCharArray(c_szColumnCapType));
    m_AcceptProps.push_back(GString::fromWCharArray(c_szColumnBaseType));
}

void GTJSpecialParamTypeWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    Q_UNUSED(pProp);
    Q_UNUSED(strNewValue);
    Q_UNUSED(strOldValue);
}

bool GTJSpecialParamTypeWriter::validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg)
{
    try
    {
        pProp->check(strValue);
    }
    catch (GMPModelException e)
    {
        strErrMsg = e.message();
        return false;
    }
    catch (...)
    {
        return false;
    }

    return true;
}

void GTJSpecialParamTypeWriter::onButtonClick(vector<GMPPropPtr>& oProps, bool& bModify)
{
    if (oProps.size() <= 0)
        return;

    int nElementTypeID = -1;

    IGMPProperties* pProps = oProps[0]->owner();
    IGMPEObject* pObj = pProps->owner();
    nElementTypeID = pObj->elementType();

    if (nElementTypeID == -1)
        return;

    GMPParamSectionInfo * pSectionInfo = new GMPParamSectionInfo();
    int nType = pProps->asInteger(nElementTypeID == etColumnCap ? c_szColumnCapType : c_szColumnBaseType);
    int nPolyId = nElementTypeID == etColumnCap ?
        gtj::getPolyIDByColumnCapType(nType) : gtj::getPolyIDByColumnBaseType(nType);

    if (GTJSelParamPolyIntf::selCommonPoly(m_pParamDB, pObj, GString(), nPolyId, pSectionInfo))
    {
        // BEGIN: FIXED BUG 4078:选择多个参数化点式图元，无法修改为同一类型
        IGMPProperties* pTempProps = NULL;
        if (nElementTypeID == etColumnCap)
        {
            for (auto iter = oProps.begin(); iter != oProps.end(); iter++)
            {
                pTempProps = (*iter)->owner();
                if (pTempProps)
                {
                    pTempProps->setAsInteger(c_szColumnCapType, gtj::getColumnCapTypeByPolyID(pSectionInfo->nPolyID));
                }
            }
        }
        else if (nElementTypeID == etColumnBase)
        {
            for (auto iter = oProps.begin(); iter != oProps.end(); iter++)
            {
                pTempProps = (*iter)->owner();
                if (pTempProps)
                {
                    pTempProps->setAsInteger(c_szColumnBaseType, gtj::getColumnBaseTypeByPolyID(pSectionInfo->nPolyID));
                }
            }
        }
        // END
        bModify = true;
    }
    else
    {
        bModify = false;
    }
    delete pSectionInfo;
}

GTJRaftSlabElevPropWriter::GTJRaftSlabElevPropWriter(vector<int> * pAcceptEntTypes /*= nullptr*/)
    :GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.push_back(GString::fromStdWString(pfnTopElev));
    m_AcceptProps.push_back(GString::fromStdWString(pfnBottomElev));
    m_AcceptProps.push_back(GString::fromStdWString(pfnThickness));
}

GTJRaftSlabElevPropWriter::~GTJRaftSlabElevPropWriter()
{
}

void GTJRaftSlabElevPropWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    Q_ASSERT(pProp);
    Q_ASSERT(pProp->owner());
    Q_ASSERT(pProp->owner()->hasProp(pfnThickness));
    Q_ASSERT(pProp->owner()->hasProp(pfnBottomElev));

    //获取底标高字符串
    auto getBottomElevStr = [&] (IGMPProperties* pProps) -> GString {
        GString strBottomElev;
        CCoordinates3d coord;
        IGMPElementDrawObj *const pEDO = dynamic_cast<IGMPElementDrawObj *>(pProps->owner());
        if (pEDO != nullptr)
        {
            coord = pEDO->shape()->coordinate();
        }
        if (!coord.Z.IsParallel(CVector3d::UnitZ, g_DistEpsilon)) // 如果是斜板
        {
            strBottomElev = GString::number(pProps->asDouble(pfnBottomElev) * 0.001);
        }
        else
        {
            strBottomElev = pProps->asString(pfnBottomElev);
        }
        return strBottomElev;
    };

    //修改厚度的时候联动修改顶标高
    auto setTopElev = [&] (double dThickness, IGMPProperties* pProps)-> void {
        GMPPropPtr pTopElev = pProps->propByName(pfnTopElev);
        QString strBottomElev = getBottomElevStr(pProps);
        safeSetProp(pTopElev, strBottomElev + "+" + QString::number(dThickness * 0.001));
    };

    IGMPEObject *pObject = pProp->owner()->owner();
    IGMPEObject::EObjectType objectType = pObject->type();
    IGMPElement *pElment = nullptr;
    if (objectType == IGMPEObject::eoENT)
    {
        pElment = dynamic_cast<IGMPElement *>(pObject);
    }
    else
    {
        IGMPElementDrawObj *pEDO = dynamic_cast<IGMPElementDrawObj *>(pObject);
        pElment = pEDO->element();
    }
    IGMPEdoIterator *pEDOIterator = pElment->createEdoIter(false);
    SCOPE_EXIT { delete pEDOIterator; pEDOIterator = NULL; };

    const std::wstring &sName = pProp->propName();
    const double c_dThickness = pProp->owner()->asDouble(pfnThickness) / 1000;
    if (strNewValue.trimmed().isEmpty())
    {
        if (sName == pfnTopElev)
        {
            pProp->initDefaultValue();
            pProp->owner()->propByName(pfnBottomElev)->initDefaultValue();
        }
        else if (sName == pfnBottomElev)
        {
            GMPPropPtr pTopElev = pProp->owner()->propByName(pfnTopElev);
            safeSetProp(pTopElev, qApp->translate(c_strClassName, c_strFloorBottomElev) +
                "+" + QString::number(c_dThickness));
            pProp->initDefaultValue();
        }
        else if (sName == pfnThickness)
        {
            GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
            double dThickness = pObject->floor()->slabThickness();
            if (objectType == IGMPEObject::eoENT)
            {
                setTopElev(dThickness, pProp->owner());
            }
            else
            {
                //TJGJ-28471  选中图元，修改图元厚度，构件标高也要联动
                IGMPProperties *pProps = pElment->properties();
                setTopElev(dThickness, pProps);
            }
            pEDOIterator->first();
            while (!pEDOIterator->isDone())
            {
                IGMPElementDrawObj *pEDO = pEDOIterator->curItem();
                CVector3d vec3dZ = pEDO->shape()->coordinate().Z;
                if (isEqual(vec3dZ.Z, 1.0, ggp::g_DistEpsilon))//平板
                {
                    setTopElev(dThickness, pEDO->properties());
                }
                pEDOIterator->next();
            }
        }
        return;
    }

    if (sName == pfnTopElev || sName == pfnBottomElev)
    {
        GEPData oResult;
        if (::compareValue(c_dThickness, 0.0, GMPEpsilonU) != gvrEqualsValue)
        {
            GString pStr;
            if (sName == pfnBottomElev)
            {
                pStr = strNewValue + "+" + QString::number(c_dThickness);
                safeSetProp(pProp->owner()->propByName(pfnTopElev), pStr);
            }
            else
            {
                pStr = strNewValue + "-" + QString::number(c_dThickness);
                safeSetProp(pProp->owner()->propByName(pfnBottomElev), pStr);
            }
        }
    }
    else if (sName == pfnThickness)
    {
        if (objectType == IGMPEObject::eoENT)
        {
            setTopElev(strNewValue.toDouble(), pProp->owner());
        }
        else
        {
            //TJGJ-28471  选中图元，修改图元厚度，构件标高也要联动
            IGMPProperties *pProps = pElment->properties();
            setTopElev(strNewValue.toDouble(), pProps);
        }

        pEDOIterator->first();
        while (!pEDOIterator->isDone())
        {
            IGMPElementDrawObj *pEDO = pEDOIterator->curItem();
            CVector3d vec3dZ = pEDO->shape()->coordinate().Z;
            if (isEqual(vec3dZ.Z, 1.0, ggp::g_DistEpsilon))//平板
            {
                setTopElev(strNewValue.toDouble(), pEDO->properties());
            }
            pEDOIterator->next();
        }
    }
    GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
}

void GTJRaftSlabElevPropWriter::safeSetProp(GMPPropPtr prop, const GString&sValue)
{
    //转化标高字符串
    GString strValue = GTJPropCommon::convertElevStr(sValue);
    // 单位化
    GString strTransfer = GTJPropCommon::unitTransferElev(prop, strValue, m_pUnitTransfer);
    // 根据数据类型设置值，标高目前只会是gmdtText，因为数据库中对应的是Memo
    prop->setAsString(strTransfer);
}

GTJParamSwingWinPropWriter::GTJParamSwingWinPropWriter(std::vector<int> *pAcceptEntTypes, IGMPService* pService)
    :GMPParamNAbnormalWriter(NULL, pAcceptEntTypes), m_pService(pService)
{
    m_AcceptProps.push_back(GString::fromStdWString(pfnSectionTypeID));
}

void GTJParamSwingWinPropWriter::onButtonClick(std::vector<GMPPropPtr>& oProps, bool& bModify)
{
    if (oProps.empty())
    {
        return;
    }

    int nPolyID = oProps[0]->owner()->asInteger(pfnPolyID);
    for (std::vector<GMPPropPtr>::const_iterator itr = oProps.cbegin() + 1, itrEnd = oProps.cend(); itr != itrEnd; ++itr)
    {
        if((*itr)->owner()->asInteger(pfnPolyID) != nPolyID)
        {
            return;
        }
    }

    IGMPProperties* pProps = oProps[0]->owner();
    IGMPEObject *const pObject = pProps->owner();
    if (pObject->elementType() == -1)
    {
        return;
    }

    IGMPModel *pModel = nullptr;
    if (pObject->type() == IGMPEObject::eoEDO)
    {
        pModel = dynamic_cast<IGMPElementDrawObj *>(pObject)->contnr()->model();
    }
    else if (pObject->type() == IGMPEObject::eoENT)
    {
        pModel = dynamic_cast<IGMPElement *>(pObject)->contnr()->model();
    }
    else
    {
        return;
    }

    if (esctSectParams == oProps[0]->asInteger())
    {
        GTJSwingWinWidgetDefault* pWidget = new GTJSwingWinWidgetDefault(m_pService);
        ggp::CDatabase *paramPolyDataBase = pModel->paramPolyDB();
        GTJSelParamPolyForm selParamPoly(paramPolyDataBase, pObject, pProps->asString(pfnPolyValue),
            pProps->asInteger(pfnPolyID), pWidget, QApplication::activeWindow());
        selParamPoly.setMinimumHeight(selParamPoly.height());
        if (selParamPoly.exec() == QDialog::Accepted)
        {
            pModel->beginEdit(false);
            SCOPE_EXIT { pModel->endEdit(); };
            for (size_t i = 0; i != oProps.size(); ++i)
            {
                // 设置SectionID
                pProps->setAsInteger(pfnSectionTypeID, esctSectParams);

                // 设置PolyID
                pProps->setAsInteger(pfnPolyID, selParamPoly.selPolyID());

                // 设置SectionPoly
                CPolygon* pPolygon = NULL;

                if(selParamPoly.selPolyID() == 185)
                {
                    QString strSectionPoly = selParamPoly.selSectionPoly();
                    int nPos = strSectionPoly.lastIndexOf(",");
                    double val = strSectionPoly.mid(nPos+1).toDouble();
                    if(val > 1)
                    {
                        strSectionPoly.replace(nPos+1,strSectionPoly.length()-nPos-1,"3");
                    }
                    else
                    {
                        strSectionPoly.replace(nPos+1,strSectionPoly.length()-nPos-1,"1");
                    }
                    pPolygon = str2Polygon(strSectionPoly);
                }
                else
                {
                    pPolygon = str2Polygon(selParamPoly.selSectionPoly());
                }

                if (pPolygon)
                {
                    pProps->setAsPolygon(pfnSectionPoly, *pPolygon);
                    pPolygon->Free();
                }

                // 设置插入点
                QStringList oInsertPt = selParamPoly.selInsertPoint().split(L',');
                assert(oInsertPt.count() == 2);
                pProps->setAsVector2d(pfnInsertPt, CVector2d(oInsertPt[0].toDouble(), oInsertPt[1].toDouble()));

                // 设置参数值
                pProps->setAsString(pfnPolyValue, selParamPoly.selParamValue());
                if (pProps->hasProp(L"PolyValueFormat"))
                {
                    pProps->setAsString(L"PolyValueFormat", selParamPoly.selParamValueFormat());
                }
                
                if (pProps->hasProp(pfnHeight))
                {
                    int nHight = pProps->asInteger(pfnHeight);
                    GTJSelParamPolyIntf::getParamEntHight(selParamPoly.selPolyID(), selParamPoly.paramPolyProp(), nHight);
                    pProps->setAsInteger(pfnHeight, nHight);
                }
            }
            bModify = true;
        }
        else
        {
            bModify = false;
        }
    }
}

//////////////////////////////////////////////////////////////////////////
GTJLinetelBeamWriter::GTJLinetelBeamWriter(vector<int> * pAcceptEntTypes)
    : GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.push_back(GString::fromStdWString(pfnStartPtTopElev)); //起点顶标高
    m_AcceptProps.push_back(GString::fromStdWString(pfnEndPtTopElev)); //终点顶标高
    m_AcceptProps.push_back(GString::fromStdWString(pfnGTJPosition));  //位置
    m_AcceptProps.push_back(GString::fromStdWString(pfnGTJStartPtLenInWall));  //起点深入墙长度
    m_AcceptProps.push_back(GString::fromStdWString(pfnGTJEndPtLenInWall));  //终点深入墙长度
    m_AcceptProps.push_back(GString::fromStdWString(pfnAxisOffset));//左墙皮跟中心线的距离
    m_AcceptProps.push_back(GString::fromStdWString(pfnSectionWidth));//截面宽度
    m_AcceptProps.push_back(GString::fromStdWString(pfnLen));//长度
}

bool GTJLinetelBeamWriter::updateElevOfArch(GMPPropPtr const pProp, const GString &strNewValue,
    const GString &strOldValue)
{
    IGMPProperties *const c_pProps = pProp->owner();
    IGMPEObject *const c_pObj = c_pProps->owner();
    if (IGMPEObject::eoEDO == c_pObj->type())
    {
        IGMPElementDrawObj *const c_pElmDrawObj = dynamic_cast<IGMPElementDrawObj*>(c_pObj);
        IGMPElevOperator *const c_pElevOper = c_pElmDrawObj->contnr()->model()->oprCenter()->elevOpr();
        IGMPSectionLineSolidShape *const c_pShape = dynamic_cast<IGMPSectionLineSolidShape *>(c_pElmDrawObj->shape());
        const CVector2d c_oPoint = c_pShape->line()->StartPoint();
        const int c_nArchType = c_pShape->archInfoType();

        //选中两个标高不一样的过梁时，strOldValue是？，不能用
        GString strOldTopElev = c_pElmDrawObj->properties()->propByName(pfnStartPtTopElev)->asString();
        bool bStatus = false;
        double dTemp = strOldValue.toDouble(&bStatus) * 1000;
        if (!bStatus)
        {
            if (strOldValue.trimmed().isEmpty())
            {
                dTemp = c_pElmDrawObj->properties()->propByName(pfnStartPtTopElev)->asDouble();
            }
            else
            {
                dTemp = c_pElevOper->calcElev(c_pElmDrawObj, c_oPoint, strOldTopElev);
            }
        }
        double c_dOldValue = dTemp;
        dTemp = strNewValue.toDouble(&bStatus) * 1000;
        if (!bStatus)
        {
            dTemp = c_pElevOper->calcElev(c_pElmDrawObj, c_oPoint, strNewValue);
        }
        const double c_dNewValue = dTemp;
        if (ggp::sameValue(c_dOldValue, c_dNewValue, ggp::g_DistEpsilon))
        {
            GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldTopElev);
            return true;
        }
        else if (c_nArchType == laiRingBeam)
        {
            if (GMPMessageBox::question(QApplication::activeWindow(),
                qApp->translate(c_szPropInfoWriter, c_Confrim), qApp->translate(c_szPropInfoWriter, c_ArchElevModifyHintInfo),
                GlodonMessageBox::No | GlodonMessageBox::Yes, GlodonMessageBox::No) == GlodonMessageBox::Yes)
            {
                c_pShape->setArchInfoType(laiPlane);
                GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldTopElev);
                return true;
            }
        }
        else if (c_nArchType == laiBeam)
        {
            GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldTopElev);
            return true;
        }
    }
    return false;
}

// 判断修改起/始点深入墙的距离后是否超过圆周长
bool GTJLinetelBeamWriter::isOverCircumference(CCurve2d* pLine, int nPtLenInWall, int nNewValue)
{
    if (Arc2dType != pLine->Type())
    {
        return false;
    }
    const double dLen = pLine->Length();
    CArc2d* oArcLine = dynamic_cast<CArc2d *>(pLine);
    if (nullptr == oArcLine || oArcLine->IsClosed())
    {
        return false;
    }
    const double dRadius = oArcLine->Radius();
    const double dCircumference = 2 * M_PI * dRadius;
    if (IsGreaterThan(dLen - nPtLenInWall + nNewValue, dCircumference, g_DistEpsilon))
    {
        return true;
    }
    return false;
}

void GTJLinetelBeamWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    auto safeSetProp = [&] (GMPPropPtr prop, const GString&sValue) {
        GString strTransfer = GTJPropCommon::unitTransferElev(prop, sValue, m_pUnitTransfer);
        prop->setAsString(strTransfer);
    };

    IGMPProperties *const c_pProps = pProp->owner();
    IGMPEObject *const c_pObj = c_pProps->owner();
    if (c_pObj->type() == IGMPEObject::eoEDO)
    {
        IGMPElementDrawObj *pLintelEDO = dynamic_cast<IGMPElementDrawObj *>(c_pObj);
        pLintelEDO->properties()->setAsBoolean(pfnDoRingBeam, true);
    }
    else
    {
        IGMPElement* pLintelElement = dynamic_cast<IGMPElement *>(c_pObj);
        IGMPEdoIterator *pLintelEdoIter = pLintelElement->createEdoIter(false);
        pLintelEdoIter->first();
        while (!pLintelEdoIter->isDone())
        {
            IGMPElementDrawObj *pEDO = pLintelEdoIter->curItem();
            pEDO->properties()->setAsBoolean(pfnDoRingBeam, true);
            pLintelEdoIter->next();
        }
        delete pLintelEdoIter;
    }
    if (canResumeDefaultValue(pProp, strNewValue))
    {
        if (pProp->propName() == pfnStartPtTopElev)
        {
            IGMPEObject *c_pEObject = pProp->owner()->owner();
            if (c_pEObject->type() == IGMPEObject::eoEDO)
            {
                IGMPElementDrawObj *pEDO = dynamic_cast<IGMPElementDrawObj *>(c_pEObject);
                const auto pShape = dynamic_cast<IGMPCustomLineSolidShape *>(pEDO->shape());
                if (pShape->archInfoType() == laiRingBeam)
                {
                    if (GMPMessageBox::question(QApplication::activeWindow(),
                        qApp->translate(c_szPropInfoWriter, c_Confrim), qApp->translate(c_szPropInfoWriter, c_ArchElevModifyHintInfo),
                        GlodonMessageBox::No | GlodonMessageBox::Yes, GlodonMessageBox::No) == GlodonMessageBox::Yes)
                    {
                        pShape->setArchInfoType(laiPlane);
                    }
                }
                else
                {
                    pProp->initDefaultValue();
                }
            }
            else
            {
                pProp->initDefaultValue();
            }
        }
        else
        {
            pProp->initDefaultValue();
        }
        if (pProp->propName() == pfnStartPtTopElev)
        {
            GString oStr("0");
            safeSetProp(pProp->owner()->propByName(pfnStartPtTopElev), oStr);
            pProp->owner()->propByName(pfnGTJPosition)->setAsString(NULL);
            safeSetProp(pProp->owner()->propByName(pfnEndPtTopElev),
                qApp->translate("GGJLinetelBeamWriter", c_strLinetelHoleTopElevAddBeamHeight));
            safeSetProp(pProp->owner()->propByName(pfnStartPtTopElev),
                qApp->translate("GGJLinetelBeamWriter", c_strLinetelHoleTopElevAddBeamHeight));
            return;
        }
        else if (pProp->propName() == pfnGTJPosition)
        {
            if (c_pObj->type() == IGMPEObject::eoEDO)
            {
                const auto pEDO = dynamic_cast<IGMPElementDrawObj *>(c_pObj);
                IGMPCustomLineSolidShape *pShape = dynamic_cast<IGMPCustomLineSolidShape *>(pEDO->shape());
                //若修改之前是洞口下方，清空后要恢复成洞口上方，清空前是洞口上方不需处理
                if (strOldValue.toInt() == 1 && pShape->archInfoType() == laiRingBeam)
                {
                    pShape->setArchInfoType(laiPlane);
                }
            }
            safeSetProp(pProp->owner()->propByName(pfnStartPtTopElev),
                qApp->translate("GGJLinetelBeamWriter", c_strLinetelHoleTopElevAddBeamHeight));
            safeSetProp(pProp->owner()->propByName(pfnEndPtTopElev),
                qApp->translate("GGJLinetelBeamWriter", c_strLinetelHoleTopElevAddBeamHeight));
            return;
        }
        return;
    }
    const std::wstring &c_wsName = pProp->propName();
    if (c_wsName == pfnGTJPosition)
    {
        //联动标高
        if (c_pObj->type() == IGMPEObject::eoEDO)
        {
            const auto pEDO = dynamic_cast<IGMPElementDrawObj *>(c_pObj);
            IGMPCustomLineSolidShape *pShape = dynamic_cast<IGMPCustomLineSolidShape *>(pEDO->shape());
            if (pShape->archInfoType() == laiRingBeam)
            {
                pShape->setArchInfoType(laiPlane);
            }
        }
        if (strNewValue.toInt() == LINETEL_HOLD_UP)
        {
            safeSetProp(pProp->owner()->propByName(pfnStartPtTopElev),
                qApp->translate("GGJLinetelBeamWriter", c_strLinetelHoleTopElevAddBeamHeight));
            safeSetProp(pProp->owner()->propByName(pfnEndPtTopElev),
                qApp->translate("GGJLinetelBeamWriter", c_strLinetelHoleTopElevAddBeamHeight));
        }
        else if (strNewValue.toInt() == LINETEL_HOLD_DOWN)
        {
            safeSetProp(pProp->owner()->propByName(pfnStartPtTopElev),
                qApp->translate("GGJLinetelBeamWriter", c_strLinetelHoleBottomElev));
            safeSetProp(pProp->owner()->propByName(pfnEndPtTopElev),
                qApp->translate("GGJLinetelBeamWriter", c_strLinetelHoleBottomElev));
        }
        safeSetProp(pProp, strNewValue);
    }
    else if (c_wsName == pfnStartPtTopElev)
    {
        //联动标高
        IGMPEObject *c_pEObject = pProp->owner()->owner();
        if (c_pEObject->type() == IGMPEObject::eoENT)
        {
            safeSetProp(pProp->owner()->propByName(pfnEndPtTopElev), strNewValue);
            safeSetProp(pProp, strNewValue);
        }
        else
        {
            IGMPElementDrawObj *pEDO = dynamic_cast<IGMPElementDrawObj *>(c_pEObject);
            IGMPSectionLineSolidShape *pShape = dynamic_cast<IGMPSectionLineSolidShape *>(pEDO->shape());
            if (pShape->archInfoType() == laiPlane)
            {
                safeSetProp(pProp->owner()->propByName(pfnEndPtTopElev), strNewValue);
                safeSetProp(pProp, strNewValue);
            }
        }
        //联动洞口位置
        if (strNewValue == qApp->translate("GGJLinetelBeamWriter", c_strLinetelHoleTopElevAddBeamHeight))
        {
            safeSetProp(pProp->owner()->propByName(pfnGTJPosition), GString("%1").arg(LINETEL_HOLD_UP));
        }
        else if (strNewValue == qApp->translate("GGJLinetelBeamWriter", c_strLinetelHoleBottomElev))
        {
            safeSetProp(pProp->owner()->propByName(pfnGTJPosition), GString("%1").arg(LINETEL_HOLD_DOWN));
        }
        if (updateElevOfArch(pProp, strNewValue, strOldValue))
        {
            safeSetProp(pProp->owner()->propByName(pfnEndPtTopElev), strNewValue);
            safeSetProp(pProp, strNewValue);
        }
    }
    else if (c_wsName == pfnEndPtTopElev)
    {
        //联动标高
        safeSetProp(pProp->owner()->propByName(pfnStartPtTopElev), strNewValue);

        //联动洞口位置
        if (strNewValue == qApp->translate("GGJLinetelBeamWriter", c_strLinetelHoleTopElevAddBeamHeight))
        {
            safeSetProp(pProp->owner()->propByName(pfnGTJPosition), GString("%1").arg(LINETEL_HOLD_UP));
        }
        else if (strNewValue == qApp->translate("GGJLinetelBeamWriter", c_strLinetelHoleBottomElev))
        {
            safeSetProp(pProp->owner()->propByName(pfnGTJPosition), GString("%1").arg(LINETEL_HOLD_DOWN));
        }
        safeSetProp(pProp, strNewValue);
    }

    // 以下处理过梁入墙长度
    if (c_wsName == pfnGTJStartPtLenInWall)
    {
        IGMPProperties* pProps = pProp->owner();
        IGMPEObject* pObj = pProps->owner();

        IGMPElementDrawObj* pEDO = nullptr;
        if (IGMPEObject::eoEDO == pObj->type())
        {
            pEDO = dynamic_cast<IGMPElementDrawObj*>(pObj);
        }
        else if (IGMPEObject::eoENT == pObj->type())
        {
            safeSetProp(pProp, strNewValue);
            return;
        }
        safeSetProp(pProp, strNewValue);
        IGMPSectionLineSolidShape* pLineShape = dynamic_cast<IGMPSectionLineSolidShape*>(pEDO->shape());
        CCurve2d* pLine = pLineShape->line()->Clone();
        SCOPE_EXIT { delete pLine; };
        int nChangeValue = strNewValue.toInt() - strOldValue.toInt();
        if (nChangeValue > 0)
            pLine->Extend(nChangeValue, false);  //起点的反响延长
        else
            pLine->SetStartT(pLine->StartT() - nChangeValue);  //起点的正向缩短

        //设置过梁长度
        //GString strLength = GString("%1").arg(pLine->Length());
        //safeSetProp(pProp->owner()->propByName(pfnLen), strLength);
    }
    else if (c_wsName == pfnGTJEndPtLenInWall)
    {
        IGMPProperties* pProps = pProp->owner();
        IGMPEObject* pObj = pProps->owner();

        IGMPElementDrawObj* pEDO = nullptr;
        if (IGMPEObject::eoEDO == pObj->type())
        {
            pEDO = dynamic_cast<IGMPElementDrawObj*>(pObj);
        }
        else if (IGMPEObject::eoENT == pObj->type())
        {
            safeSetProp(pProp, strNewValue);
            return;
        }
        safeSetProp(pProp, strNewValue);
        IGMPSectionLineSolidShape* pLineShape = dynamic_cast<IGMPSectionLineSolidShape*>(pEDO->shape());
        CCurve2d* pLine = pLineShape->line()->Clone();
        SCOPE_EXIT { delete pLine; };

        int nChangeValue = strNewValue.toInt() - strOldValue.toInt();
        if (nChangeValue > 0)
            pLine->Extend(nChangeValue, true);  //终点的正向延长
        else
            pLine->SetEndT(pLine->EndT() + nChangeValue);  //终点的反向缩短

        //设置过梁长度
        //GString strLength = GString("%1").arg(pLine->Length());
        //safeSetProp(pProp->owner()->propByName(pfnLen), strLength);
    }
    else if (c_wsName == pfnAxisOffset)
    {
        GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
    }
    else if (c_wsName == pfnSectionWidth)
    {
        IGMPProperties* pProps = pProp->owner();
        IGMPEObject* pObj = pProps->owner();
        GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
        if (IGMPEObject::eoEDO == pObj->type())
        {
            IGMPElementDrawObj *pEDO = dynamic_cast<IGMPElementDrawObj*>(pObj);
            IGMPRelationOperator *pRelaOpr = pEDO->contnr()->model()->oprCenter()->relaOpr();
            IGMPElementDrawObj *pParentEDO = pRelaOpr->parent(pEDO);
            const double dWallWidth = getLintelWallWidth(pParentEDO);
            const double dSectionWidth = pEDO->properties()->asDouble(pfnSectionWidth);
            const double dAxisOffset = pEDO->properties()->asDouble(pfnAxisOffset);
            if (IsGreaterEqualThan(dAxisOffset, dSectionWidth * 0.5 + dWallWidth, g_DistEpsilon)
                || IsLessEqualThan(dAxisOffset, -dSectionWidth * 0.5, g_DistEpsilon))
            {
                pEDO->properties()->propByName(pfnAxisOffset)->setAsDouble(dWallWidth * 0.5);
            }
        }
    }
    else if (c_wsName == pfnLen)
    {
        IGMPProperties* pProps = pProp->owner();
        IGMPEObject* pObj = pProps->owner();
        if (IGMPEObject::eoENT == pObj->type())
        {
            GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
            return;
        }

        IGMPElementDrawObj* pEDO = dynamic_cast<IGMPElementDrawObj*>(pObj);
        if (pEDO == nullptr)
        {
            return;
        }
        const int nStartInWall = pEDO->properties()->asInteger(pfnGTJStartPtLenInWall);
        const int nEndInWall = pEDO->properties()->asInteger(pfnGTJEndPtLenInWall);
        IGMPElementDrawObj *pParentEDO = pEDO->contnr()->model()->oprCenter()->relaOpr()->parent(pEDO);
        //取得过梁父图元的洞口长度
        const int nOpenWidth = (int) getLintelParentOpeningWidth(pParentEDO);
        if ((nStartInWall <= 0 && nEndInWall <= 0))
        {
            GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
            return;
        }
        int nLastIndex = strOldValue.indexOf(")");
        GString strMyOldValue = strOldValue;
        if (nLastIndex > 0)
        {
            strMyOldValue = strMyOldValue.right(strMyOldValue.length() - 1);
            strMyOldValue = strMyOldValue.left(strMyOldValue.length() - 1);
        }
        if (strNewValue == "" || IsGreaterEqualThan(strNewValue.toInt(), strMyOldValue.toInt(), g_DistEpsilon))
        {
            GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strMyOldValue);
            return;
        }
        //只考虑长度减小，并且左右伸入墙内长度有数值的情况
        if (IsLessEqualThan(strNewValue.toInt(), nOpenWidth, g_DistEpsilon))
        {
            //int nReduce = (nOpenWidth - (strNewValue.toInt() - nStartInWall - nEndInWall))/2;
            pEDO->properties()->setAsInteger(pfnGTJStartPtLenInWall, 0);
            pEDO->properties()->setAsInteger(pfnGTJEndPtLenInWall, 0);
        }
        else
        {
            const int nInWallLen = strNewValue.toInt() - nOpenWidth;
            bool bStart = (nStartInWall > nEndInWall) ? true : false;
            if (bStart)
            {
                if (nStartInWall - nEndInWall < nInWallLen)
                {
                    pEDO->properties()->setAsInteger(pfnGTJStartPtLenInWall, (nInWallLen + nStartInWall - nEndInWall) / 2);
                    pEDO->properties()->setAsInteger(pfnGTJEndPtLenInWall, (nInWallLen - nStartInWall + nEndInWall) / 2);
                }
                else
                {
                    pEDO->properties()->setAsInteger(pfnGTJStartPtLenInWall, nInWallLen);
                    pEDO->properties()->setAsInteger(pfnGTJEndPtLenInWall, 0);
                }
            }
            else
            {
                if (nEndInWall - nStartInWall < nInWallLen)
                {
                    pEDO->properties()->setAsInteger(pfnGTJEndPtLenInWall, (nInWallLen + nStartInWall - nEndInWall) / 2);
                    pEDO->properties()->setAsInteger(pfnGTJStartPtLenInWall, (nInWallLen - nStartInWall + nEndInWall) / 2);
                }
                else
                {
                    pEDO->properties()->setAsInteger(pfnGTJEndPtLenInWall, nInWallLen);
                    pEDO->properties()->setAsInteger(pfnGTJStartPtLenInWall, 0);
                }
            }
        }
        GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
    }
}

bool GTJLinetelBeamWriter::validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg)
{
    if (pProp->propName() == pfnStartPtTopElev)
    {
        return GMPPropInfoDefaultWriter::validateProp(pProp, strValue, strErrMsg);
    }
    //顶标高，位置如果设置为空的话，不抛异常，直接返回之前的值
    if (canResumeDefaultValue(pProp, strValue))
    {
        return true;
    }

    //modify by liujx Fix.bug TJGJ-20378 和 TJGJ-19981
    IGMPEObject* pObj = pProp->owner()->owner();
    if (pObj->type() == IGMPEObject::eoEDO)
    {
        if ((pProp->propName() == pfnSectionWidth) && pProp->isDataNull())
        {
            return true;
        }
    }
    else if (pObj->type() == IGMPEObject::eoENT)
    {
        static bool bValid = false;
        if (strValue.isEmpty())
        {
            bValid = true;
        }
        else if (strValue == "-1" && bValid == true)
        {
            bValid = false;
            return true;
        }
        else
        {
            bValid = false;
        }
    }

    try
    {
        bool bFind = true;
        if (!GTJPropCommon::isDigitStr(strValue))
        {
            QString strPickList = pProp->schema()->pickList();
            if (strPickList.length() > 0 && strPickList.at(0) == '!')
            {
                strPickList = const_cast<IGMPProperties*>(pProp->owner())->evaluate(strPickList.mid(1)).asString();
            }

            bFind = false;
            QStringList strList = strPickList.split('\n');
            for (int i = 0; i < strList.count(); i++)
            {
                QString strResult = strList[i].trimmed();
                if (strValue.lastIndexOf(strResult) >= 0)
                {
                    bFind = true;
                }
            }
        }

        if (!bFind)
        {
            pProp->check("*****");
        }
        else
        {
            pProp->check(strValue);
        }
    }
    catch (GMPModelException &e)
    {
        strErrMsg = e.message();
        return false;
    }
    if (pProp->propName() == pfnAxisOffset)
    {
        bool bOk = true;
        if (strValue.isEmpty())//直接清空的话，不弹框默认恢复为默认值
        {
            return true;
        }
        double dCenterAxisOffset = strValue.toDouble(&bOk);
        if (!bOk)
        {
            strErrMsg = QString::fromLocal8Bit(c_strInvalidNum).arg(strValue);
            return false;
        }
        if (IGMPEObject::eoENT == pProp->owner()->owner()->type())
        {
            return true;
        }
        //对于超出范围的过梁中心线距左墙皮距离的输入，判断时的优先级按照砌体墙>剪力墙>保温墙>栏板，同一构件按照ID由小到大
        IGMPElementDrawObj *pEDO = dynamic_cast<IGMPElementDrawObj *>(pProp->owner()->owner());
        IGMPModel *pModel = pEDO->contnr()->model();
        IGMPElementDrawObj *pParentEdo = pModel->oprCenter()->relaOpr()->parent(pEDO);
        //按优先级排序后的顺序
        vector<IGMPElementDrawObj *> oWallVector;
        getWallByLintelParent(oWallVector, pEDO, pParentEdo);
        const int nParentELement = pParentEdo->elementType();
        //若过梁的父为门窗洞先判断是否超出了其爷爷的范围，若超出直接提示错误信息
        switch (nParentELement)
        {
            case etDoor:
            case etWindow:
            case etDoorWin:
            case etOpening:
            case etAlcove:
            {
                IGMPElementDrawObj *pTempPPEdo = pModel->oprCenter()->relaOpr()->parent(pParentEdo);
                if (pTempPPEdo == nullptr)
                {
                    break;
                }
                IGMPCustomLineSolidShape* pLineShape = dynamic_cast<IGMPCustomLineSolidShape*>(pTempPPEdo->shape());
                double dWallThickness = pLineShape->lineWidth();
                bool bIsValid = isValidAxisOffset(pEDO, dWallThickness, dCenterAxisOffset, strErrMsg);
                if (!bIsValid)
                {
                    return false;
                }
                break;
            }
            default:
                break;
        }
        for (auto it = oWallVector.begin(); it != oWallVector.end(); ++it)
        {
            //上面已经判断过过梁在门窗洞上时，它的爷爷，此处不需要再判断
            if ((*it) == pModel->oprCenter()->relaOpr()->parent(pParentEdo))
            {
                continue;
            }
            double dWallThickness(0);
            if ((*it)->elementType() == etParapet)
            {
                IGMPSectionLineSolidShape *pLineShae = dynamic_cast<IGMPSectionLineSolidShape*>((*it)->shape());
                dWallThickness = pLineShae->lineWidth();
            }
            else
            {
                dWallThickness = (*it)->properties()->asDouble(pfnThickness);
            }
            bool bValid = isValidAxisOffset(pEDO, dWallThickness, dCenterAxisOffset, strErrMsg);
            if (!bValid)
            {
                return false;
            }
        }
    }
    if (pProp->propName() == pfnGTJStartPtLenInWall || pProp->propName() == pfnGTJEndPtLenInWall)
    {
        IGMPProperties* pProps = pProp->owner();
        IGMPEObject* pObj = pProps->owner();
        IGMPElementDrawObj* pEDO = nullptr;
        if (IGMPEObject::eoEDO == pObj->type())
        {
            pEDO = dynamic_cast<IGMPElementDrawObj*>(pObj);
        }
        else if (IGMPEObject::eoENT == pObj->type())
        {
            return true;
        }
        IGMPSectionLineSolidShape* pLineShape = dynamic_cast<IGMPSectionLineSolidShape*>(pEDO->shape());
        CCurve2d* pLine = pLineShape->line()->Clone();
        SCOPE_EXIT { delete pLine; };
        int nPtLenInWall = pProp->asInteger();
        if (isOverCircumference(pLine, nPtLenInWall, strValue.toInt()))
        {
            strErrMsg = QString::fromLocal8Bit(c_strLinetelPtLenInWall);
            return false;
        }
    }
    return true;
}

bool GTJLinetelBeamWriter::isValidAxisOffset(IGMPElementDrawObj* pEDO, double& dWallThickness, double& dCenterAxisOffset, GString& strErrMsg)
{
    double dSectionWidth(0);
    if (pEDO->properties()->isDataNull(pfnSectionWidth))
    {
        IGMPSectionLineSolidShape *pShape = dynamic_cast<IGMPSectionLineSolidShape *>(pEDO->shape());
        dSectionWidth = pShape->lineWidth();
    }
    else
    {
        dSectionWidth = pEDO->properties()->asDouble(pfnSectionWidth);
    }
    if ((dCenterAxisOffset >= dSectionWidth / 2 + dWallThickness)
        || (dCenterAxisOffset <= -dSectionWidth / 2))
    {
        strErrMsg = QString::fromLocal8Bit(c_strLinetelAxisOffset).arg(-dSectionWidth / 2).arg(dSectionWidth / 2 + dWallThickness);
        return false;
    }
    return true;
}

void GTJLinetelBeamWriter::getWallByLintelParent(vector<IGMPElementDrawObj*> &oWallVector, IGMPElementDrawObj *pEDO, IGMPElementDrawObj *pParentEdo)
{
    IGMPElementTypeSet oSet;
    oSet.insert(gtj::etForceWall);
    oSet.insert(gtj::etBrickWall);
    oSet.insert(etInsulatingWall);
    oSet.insert(etParapet);
    IGMPEdoIterator *pEDOIterator = pEDO->contnr()->model()->edoContnr()->createIter(&oSet, pParentEdo->shape()->box());
    SCOPE_EXIT { delete pEDOIterator; };
    CBodyPtr pParentBody = nullptr;
    IGMPSectionPointSolidShape *pPointShape = dynamic_cast<IGMPSectionPointSolidShape *>(pParentEdo->shape());
    if (pPointShape == nullptr)
    {
        IGMPLineSolidShape* pLineShape = dynamic_cast<IGMPLineSolidShape*>(pParentEdo->shape());
        pParentBody = pLineShape->body();
    }
    else
    {
        pParentBody = pPointShape->body();
    }
    vector<IGMPElementDrawObj *> oWall1;            //剪力墙
    vector<IGMPElementDrawObj *> oWall2;            //砌体墙
    vector<IGMPElementDrawObj *> oInsulatingWall;   //保温墙
    vector<IGMPElementDrawObj *> oParapet;          //栏板
    for (pEDOIterator->first(); !pEDOIterator->isDone();)
    {
        //由于带型窗的box比较大，因此迭代器中有些图元可能是不需要的，此处通过体相交过滤到无用的墙图元
        IGMPCustomLineSolidShape *pWallShape = dynamic_cast<IGMPCustomLineSolidShape *>(pEDOIterator->curItem()->shape());
        CBodyPtr pWallBody = pWallShape->body();
        BodyPosition oBodyPosition = CPositionJudge().GetBodyPosition(pParentBody.get(), pWallBody.get());
        if (oBodyPosition != bpIntersection)
        {
            pEDOIterator->next();
            continue;
        }

        if (pEDOIterator->curItem()->elementType() == gtj::etForceWall)
        {
            oWall1.push_back(pEDOIterator->curItem());
        }
        else if (pEDOIterator->curItem()->elementType() == etInsulatingWall)
        {
            oInsulatingWall.push_back(pEDOIterator->curItem());
        }
        else if (pEDOIterator->curItem()->elementType() == etParapet)
        {
            oParapet.push_back(pEDOIterator->curItem());
        }
        else
        {
            oWall2.push_back(pEDOIterator->curItem());
        }
        pEDOIterator->next();
    }
    //对各个构件再进行id由小到大的排序
    auto isLessId = [&] (IGMPElementDrawObj* pEdo1, IGMPElementDrawObj* pEdo2)->bool {
        return pEdo1->iD() < pEdo2->iD();
    };
    std::sort(oWall1.begin(), oWall1.end(), isLessId);
    std::sort(oWall2.begin(), oWall2.end(), isLessId);
    std::sort(oInsulatingWall.begin(), oInsulatingWall.end(), isLessId);
    std::sort(oParapet.begin(), oParapet.end(), isLessId);
    oWallVector.insert(oWallVector.end(), oWall2.begin(), oWall2.end());
    oWallVector.insert(oWallVector.end(), oWall1.begin(), oWall1.end());
    oWallVector.insert(oWallVector.end(), oInsulatingWall.begin(), oInsulatingWall.end());
    oWallVector.insert(oWallVector.end(), oParapet.begin(), oParapet.end());
}

GTJPostCastStripTypeWriter::GTJPostCastStripTypeWriter(ggp::CDatabase* pParamDB, vector<int> *pAcceptEntTypes /*= nullptr*/)
    : GMPPropInfoWriter(pAcceptEntTypes), m_pParamDB(pParamDB)
{
    m_AcceptProps.push_back(GString::fromStdWString(GTJDBCConsts::pfnRaftSectionTypeID));
    m_AcceptProps.push_back(GString::fromStdWString(GTJDBCConsts::pfnSlabSectionTypeID));
    m_AcceptProps.push_back(GString::fromStdWString(GTJDBCConsts::pfnWallOutSectionTypeID));
    m_AcceptProps.push_back(GString::fromStdWString(GTJDBCConsts::pfnWallInSectionTypeID));
    m_AcceptProps.push_back(GString::fromStdWString(GTJDBCConsts::pfnBeamSectionTypeID));
    m_AcceptProps.push_back(GString::fromStdWString(GTJDBCConsts::pfnFDBeamSectionTypeID));
}

void GTJPostCastStripTypeWriter::writeProp(GMPPropPtr pProp,
    const GString& strNewValue, const GString& strOldValue)
{
    Q_UNUSED(pProp);
    Q_UNUSED(strNewValue);
    Q_UNUSED(strOldValue);
}

bool GTJPostCastStripTypeWriter::validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg)
{
    try
    {
        pProp->check(strValue);
    }
    catch (GMPModelException e)
    {
        strErrMsg = e.message();
        return false;
    }
    catch (...)
    {
        return false;
    }
    return true;
}

void GTJPostCastStripTypeWriter::onButtonClick(vector<GMPPropPtr>& oProps, bool& bModify)
{
    if (oProps.size() <= 0)
        return;

    IGMPProperties* pProps = oProps[0]->owner();
    IGMPEObject* pObj = pProps->owner();
    int nElementTypeID = pObj->elementType();

    if (nElementTypeID == -1)
        return;

    GTJPostCastStripParamWidget *pCustomWidget =
        dynamic_cast<GTJPostCastStripParamWidget*>(GTJSelParamPolyIntf::createPostCastStripParamWidget(oProps[0]));
    GTJSelParamPolyForm selParamPoly(m_pParamDB, nElementTypeID, pCustomWidget, QApplication::activeWindow());
    selParamPoly.setMinimumHeight(selParamPoly.height());
    if (selParamPoly.exec() == QDialog::Accepted)
    {
        return;
    }
}

GTJSumpSlopInfoWriter::GTJSumpSlopInfoWriter(vector<int> * pAcceptEntTypes/* = nullptr*/)
    : GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.push_back(GString::fromStdWString(GTJDBCConsts::pfnOutside));
    m_AcceptProps.push_back(GString::fromStdWString(GTJDBCConsts::pfnAngle));
    m_AcceptProps.push_back(GString::fromStdWString(GTJDBCConsts::pfnBottomWide));
    m_AcceptProps.push_back(GString::fromStdWString(GTJDBCConsts::pfnSlopeInput));
}

GTJSumpSlopInfoWriter::~GTJSumpSlopInfoWriter()
{
}

bool GTJSumpSlopInfoWriter::validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg)
{
    bool bRes = true;
    try
    {
        if (pProp->propName() == pfnAngle)
        {
            // 针对放坡角度进行特殊处理
            pProp->check(strValue);
        }
        else
        {
            bRes = GMPPropInfoDefaultWriter::validateProp(pProp, strValue, strErrMsg);
        }
    }
    catch (const GMPModelException &except)
    {
        strErrMsg = except.message();
        bRes = false;
    }
    return bRes;
}

void GTJSumpSlopInfoWriter::writeProp(GMPPropPtr pProp, const GString &strNewValue, const GString &strOldValue)
{
    GString strTmpValue = strNewValue;
    //增加放坡角度输入四舍五入保留两位小数
    auto const pPropName = pProp->propName();
    if (0 == pPropName.compare(GTJDBCConsts::pfnAngle))
    {
        if (!strNewValue.isEmpty())
        {
            strTmpValue = QString::number(Round(strTmpValue.toDouble() * 1000) * 0.001, 'f', 3);
        }
    }

    // 属性值没改变时，不处理
    if (strTmpValue == strOldValue)
    {
        return;
    }

    if (strTmpValue.isEmpty())
    {
        if (pPropName.compare(GTJDBCConsts::pfnAngle) == 0)
        {
            strTmpValue.setNum((strOldValue == "?") ? 0.1 : 45);
        }
        else if (pPropName.compare(GTJDBCConsts::pfnOutside) == 0)
        {
            strTmpValue.setNum((strOldValue == "?") ? 1 : 500);
        }
        else if (pPropName.compare(GTJDBCConsts::pfnBottomWide) == 0)
        {
            strTmpValue.setNum((strOldValue == "?") ? 1 : 600);
        }
        pProp->setAsString(strTmpValue);
    }

    // 调用父类接口，写入属性
    GMPPropInfoDefaultWriter::writeProp(pProp, strTmpValue, strOldValue);
    IGMPElementDrawObj *pSumpEDO = dynamic_cast<IGMPElementDrawObj*>(pProp->owner()->owner());
    // 当前编辑的属性是集水坑图元的属性，需要同时更新图元的边坡信息
    if (pSumpEDO != nullptr)
    {
        IGMPModel * pModel = pSumpEDO->contnr()->model();
        pModel->calculate();
        writeEdgeInfos(pSumpEDO, pProp->propName());
    }
}

void GTJSumpSlopInfoWriter::writeEdgeInfos(IGMPElementDrawObj *pSumpEDO, const wstring &strPropName)
{
    assert(pSumpEDO != nullptr);
    IGMPSolidShape *pSolidShape = dynamic_cast<IGMPSolidShape*>(pSumpEDO->shape());
    if (pSolidShape == nullptr)
    {
        assert(false);
        return;
    }

    // 已经在writeProp函数内，调用了父类的同名虚函数将修改的属性写入了图元
    // 此时获取图元的边坡信息，取得的边坡信息是最新的
    GString strEDOVar,
        strEDOOutWidth = pSumpEDO->properties()->asString(GTJDBCConsts::pfnOutside);
    int nSlopeInput = pSumpEDO->properties()->asInteger(GTJDBCConsts::pfnSlopeInput);
    switch (nSlopeInput)
    {
        // 放坡角度
        case 0:
            strEDOVar = pSumpEDO->properties()->asString(GTJDBCConsts::pfnAngle);
            break;

            // 放坡底宽
        case 1:
            strEDOVar = pSumpEDO->properties()->asString(GTJDBCConsts::pfnBottomWide);
            break;

        default:
            assert(false);
            break;
    }

    IGMPEdgeInfo *pEdgeInfo = nullptr;
    IGMPEdgeInfos *pEdgeInfos = pSolidShape->getEdgeInfos();
    CPolygonPtr pWorldPoly = pSolidShape->worldPoly();
    assert(pWorldPoly->LoopCount() == 1);

    QString strEdgeInfo;
    for (int i = 0; i < pWorldPoly->GetLoop(0)->CoedgeCount(); ++i)
    {
        // 集水坑worldpoly只有一个环，所以getItem第二个参数为0
        // 边已经设置边坡信息时，更新边坡信息
        pEdgeInfo = pEdgeInfos->getItem(i, 0);
        if (pEdgeInfo)
        {
            GStringList strListParam = pEdgeInfo->paramStr().split(';');
            // 集水坑边坡信息只有两个；出边距离--放坡底宽/放坡角度
            assert(strListParam.size() == 2);

            // 当前修改的是出边距离
            if (strPropName == wstring(GTJDBCConsts::pfnOutside))
            {
                // 避免相同信息写入，数据库改变（即使是相同的信息）会触发造体
                if (strEDOOutWidth != strListParam[0])
                {
                    pEdgeInfo->setParamStr(strEDOOutWidth + ";" + strListParam[1]);
                }
            }
            // 当前修改的是放坡底宽/放坡角度
            // 当前切换了输入模式
            else if (((strPropName.compare(GTJDBCConsts::pfnAngle) == 0)
                || (strPropName.compare(GTJDBCConsts::pfnBottomWide) == 0))
                || (strPropName.compare(GTJDBCConsts::pfnSlopeInput) == 0))
            {
                // 避免相同信息写入，数据库改变（即使是相同的信息）会触发造体
                if (strEDOVar != strListParam[1])
                {
                    pEdgeInfo->setParamStr(strListParam[0] + ";" + strEDOVar);
                }
            }
            else
            {
                assert(false);
            }
        }// end if edgeinfo != nullptr
        else
        {
            // 如果没有边坡信息插入默认的信息
            pEdgeInfos->insertEdge(i, eitEdgeSlope, &pEdgeInfo, 0);
            if (pEdgeInfo)
            {
                // 当前修改的是出边距离
                QString strWriteOutWidth("500");
                if (strPropName.compare(GTJDBCConsts::pfnOutside) == 0)
                {
                    strWriteOutWidth = strEDOOutWidth;
                }
                QString strParams = strWriteOutWidth + ";" + strEDOVar;
                pEdgeInfo->setParamStr(strParams);
            }
        }
    }// end for----worldpoly的边
}

//======================================GGJColumnBasePropInfoWriter===========================================
GGJColumnBasePropInfoWriter::GGJColumnBasePropInfoWriter(std::vector< int > * pAcceptEntTypes)
    :GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.push_back(GString::fromStdWString(L"IsEdge"));
}

void GGJColumnBasePropInfoWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    assert(pProp);
    GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
}

////////////////////////////////////////////截面宽属性设值类////////////////////////////////////////////
/*!
*@brief    构造
*@author
*@param[in]vector<int> * pAcceptEntTypes       可处理的构件类型, NULL指所有类型 ,GMPPropInfoBuilder进行深拷贝，
不负责参数生命期
*@return
*/
GTJSectionWidthWriter::GTJSectionWidthWriter(vector<int> * pAcceptEntTypes)
    :GMPSectionWidthWriter(pAcceptEntTypes)
{
    //m_AcceptProps.push_back(GString::fromStdWString(pfnSectionWidth));
}

/*!
*@brief     写截面宽、顶截面宽
*@author    qinyb 2015年01月21日
*@param[in] pProp--属性
*@param[in] strNewValue--新值
*@param[in] strOldValue--上一次值
*@return    void
*/
void GTJSectionWidthWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    bool bStatus;
    const int nValue = strNewValue.toInt(&bStatus);
    if (bStatus)
    {
        if (nValue > 0 && nValue <= 50000)
        {
            GMPSectionWidthWriter::writeProp(pProp, strNewValue, strOldValue);
            return;
        }
    }
    if (canResumeDefaultValue(pProp, strNewValue))
    {
        pProp->initDefaultValue();
    }
    else
    {
        pProp->setIsNull();
    }
}

////////////////////////////////////////////截面高属性设值类////////////////////////////////////////////
/*!
*@brief    构造
*@author   qinyb
*@param[in]vector<int> * pAcceptEntTypes       可处理的构件类型, NULL指所有类型 ,GMPPropInfoBuilder进行深拷贝，不负责参数生命期
*@return
*/
GTJSectionHeightWriter::GTJSectionHeightWriter(vector<int> * pAcceptEntTypes /*= nullptr*/)
    :GMPSectionHeightWriter(pAcceptEntTypes)
{
    //m_AcceptProps.push_back(GString::fromStdWString(pfnSectionHeight));
}

/*!
*@brief     写截面高、顶截面高
*@author    qinyb 2015年01月21日
*@param[in] pProp--属性
*@param[in] strNewValue--新值
*@param[in] strOldValue--上一次值
*@return    void
*/
void GTJSectionHeightWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    bool bStatus;
    const int nValue = strNewValue.toInt(&bStatus);
    if (bStatus)
    {
        if (nValue > 0 && nValue <= 50000)
        {
            GMPSectionHeightWriter::writeProp(pProp, strNewValue, strOldValue);
            return;
        }
    }
    if (canResumeDefaultValue(pProp, strNewValue))
    {
        pProp->initDefaultValue();
    }
    else
    {
        pProp->setIsNull();
    }
}

GTJHandrailSectionWriter::GTJHandrailSectionWriter(ggp::CDatabase* pParamDB, vector<int> * pAcceptEntTypes)
    : GMPParamNAbnormalWriter(pParamDB, pAcceptEntTypes)
{
    m_AcceptProps.push_back(GString::fromStdWString(pfnHandrailSectionPoly));
    m_AcceptProps.push_back(GString::fromStdWString(pfnParapetSectionPoly));
}

void GTJHandrailSectionWriter::onButtonClick(vector<GMPPropPtr>& oProps, bool& bModify)
{
    //具体处理逻辑
    if (oProps.empty())
    {
        return;
    }

    IGMPProperties* pProps = oProps[0]->owner();
    IGMPEObject* pTemp = pProps->owner();

    int nElementTypeID = -1;

    if (IGMPElementDrawObj* pEDO = dynamic_cast<IGMPElementDrawObj*>(pTemp))
    {
        nElementTypeID = pEDO->elementType();
    }
    else if (IGMPElement* pElement = dynamic_cast<IGMPElement*>(pTemp))
    {
        nElementTypeID = pElement->elementType();
    }

    if (-1 == nElementTypeID)
    {
        return;
    }
    QWidget * activeWidget =  QApplication::activeWindow();
    //GlodonFrame 
    GMPVectorDrawingFrmEX oEditorParent(activeWidget, true, false);
    moveWidgetToScreenCenter(&oEditorParent, GMPSystemOptions::getInstance()->getMainWindow());

    SPolygonData sPolygonData;
    IGMPProperties * tempPropertites = oProps[0]->owner();
    if (oProps[0]->propName().compare(pfnHandrailSectionPoly) == 0)
    {
        //polygonFrm.getEditPolygonFrm()->editInsertPoint(false);
        if (tempPropertites)
        {
            // 界面轴网信息，首先设置默认值
            sPolygonData.strHorzSpace = "100*10";
            sPolygonData.strVertSpace = "100*10";

            // 从属性解析轴网信息
            GMPPropPtr pSectionInfoProp = tempPropertites->propByName(pfnHandrailSectionInfo);
            if (pSectionInfoProp != NULL)
            {
                QStringList sAxisInfo = pSectionInfoProp->asString().split(';');
                if (sAxisInfo.size() == 2)
                {
                    sPolygonData.strHorzSpace = sAxisInfo.at(0);
                    sPolygonData.strVertSpace = sAxisInfo.at(1);
                }
            }
        }
    }
    else if (oProps[0]->propName().compare(pfnParapetSectionPoly) == 0)
    {
        //polygonFrm.getEditPolygonFrm()->editInsertPoint(false);
        if (tempPropertites)
        {
            // 界面轴网信息，首先设置默认值
            sPolygonData.strHorzSpace = "20*15";
            sPolygonData.strVertSpace = "100*11";

            // 从属性解析轴网信息
            GMPPropPtr pSectionInfoProp = tempPropertites->propByName(pfnParapetSectionInfo);
            if (pSectionInfoProp != NULL)
            {
                QStringList sAxisInfo = pSectionInfoProp->asString().split(';');
                if (sAxisInfo.size() == 2)
                {
                    sPolygonData.strHorzSpace = sAxisInfo.at(0);
                    sPolygonData.strVertSpace = sAxisInfo.at(1);
                }
            }
        }
    }

    //栏杆扶手没有插入点属性，所以这里默认设置为原点
    sPolygonData.oInsPoint = CVector2d(0, 0);
    sPolygonData.pPolygon = oProps[0]->asPolygon()->Clone();
    oEditorParent.setPolygonData(sPolygonData);
    oEditorParent.getEditPolygonFrm()->setInsertPointModify(false);
    if (oEditorParent.exec() == rtOk)
    {
        oEditorParent.getPolygonData(sPolygonData);
        oProps[0]->setAsPolygon(*sPolygonData.pPolygon);
        if (oProps[0]->propName().compare(pfnHandrailSectionPoly) == 0)
        {
            if (tempPropertites)
            {
                QString sAxisGridInfo = sPolygonData.strHorzSpace + ";" + sPolygonData.strVertSpace;
                tempPropertites->setAsString(pfnHandrailSectionInfo, sAxisGridInfo);
                bModify = true;
            }
        }
        else if (oProps[0]->propName().compare(pfnParapetSectionPoly) == 0)
        {
            if (tempPropertites)
            {
                QString sAxisGridInfo = sPolygonData.strHorzSpace + ";" + sPolygonData.strVertSpace;
                tempPropertites->setAsString(pfnParapetSectionInfo, sAxisGridInfo);
                bModify = true;
            }
        }
    }
}

/*!
*@file
*@brief    基槽土方属性
*@author   zhangh-t
*@date     2015年3月27日
*@remarks
*@version 1.0
*Copyright (c) 1998-2013 Glodon Corporation
*/
GCLDitchEarthPropInforWriter::GCLDitchEarthPropInforWriter(vector<int> * pAcceptEntTypes /*= nullptr*/)
    :GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.push_back(GString::fromStdWString(pfnLeftSlopeFactor));
    m_AcceptProps.push_back(GString::fromStdWString(pfnRightSlopeFactor));
    m_AcceptProps.push_back(GString::fromStdWString(pfnDepth));
    m_AcceptProps.push_back(GString::fromStdWString(pfnLeftWorkingFaceWidth));
    m_AcceptProps.push_back(GString::fromStdWString(pfnRightWorkingFaceWidth));
    m_AcceptProps.push_back(GString::fromStdWString(pfnStartPtBottomElev));
    m_AcceptProps.push_back(GString::fromStdWString(pfnEndPtBottomElev));
    m_bSuc = true;
}

GCLDitchEarthPropInforWriter::~GCLDitchEarthPropInforWriter()
{
}

QString GCLDitchEarthPropInforWriter::handleInteger(const QString& strNewValue)
{
    bool bStatus;
    const double dDepth = strNewValue.toDouble(&bStatus);
    if (bStatus)
    {
        const double dValue = floor(dDepth);
        //判断Depth是否为整数。如果是非整数，返回一个负数，让其非法
        if (ggp::sameValue(dValue, dDepth, 0.001))
        {
            return QString::number(static_cast<long>(dValue));
        }
    }
    else if (strNewValue.isEmpty())
    {
        return QString("0");
    }
    return strNewValue;
}

QString GCLDitchEarthPropInforWriter::handleValue(const QString& strNewValue)
{
    const QString strDiv("1:");
    const QString strRatio("1/");
    QString strValue = strNewValue.trimmed();
    strValue.replace(QChar('：'), ':');

    // BEGIN: MODIFY BY 2015-05-22 修改bugTJGJ-18974:输入“1：”“1/”崩溃
    if (strValue == strDiv || strValue == strRatio)
    {
        m_bSuc = false;
        return strValue;
    }
    // END

    //匹配：“1:”或 "1/"
    if (strValue.startsWith(strDiv) || strValue.startsWith(strRatio))
    {
        strValue = strValue.remove(0, 2);//去除其中”或 "1/"
    }
    else if (strValue.isEmpty())
    {
        strValue.setNum(0);
    }
    return strValue;
}

void GCLDitchEarthPropInforWriter::safeSetProp(GMPPropPtr prop, const GString&sValue)
{
    GString strTransfer = GTJPropCommon::unitTransferElev(prop, sValue, m_pUnitTransfer);
    prop->setAsString(strTransfer);
}

void GCLDitchEarthPropInforWriter::writeProp(GMPPropPtr pProp, const GString& strValue, const GString& strOldValue)
{
    //删除无效0
    GString strNewValue = GTJPropCommon::convertElevStr3(strValue);

    const std::wstring &sName = pProp->propName();
    IGMPCustomLineSolidShape* pLineShape = nullptr;
    IGMPElementDrawObj *pEarthEDO = dynamic_cast<IGMPElementDrawObj*>(pProp->owner()->owner());
    if (pEarthEDO)
    {
        pLineShape = dynamic_cast<IGMPCustomLineSolidShape*>(pEarthEDO->shape());
    }
    if (pLineShape && pLineShape->isMirror())
    {
        if (sName == pfnLeftSlopeFactor)
        {
            QString strSetValue = handleValue(strNewValue);;
            safeSetProp(pProp->owner()->propByName(pfnRightSlopeFactor), QString::number(fRound(strSetValue.toDouble(), 3, 0.001), 'f', 3));
        }
        else if (sName == pfnRightSlopeFactor)
        {
            QString strSetValue = handleValue(strNewValue);;
            safeSetProp(pProp->owner()->propByName(pfnLeftSlopeFactor), QString::number(fRound(strSetValue.toDouble(), 3, 0.001), 'f', 3));
        }
        else if (sName == pfnLeftWorkingFaceWidth || sName == pfnRightWorkingFaceWidth)
        {
            WriteWorkingFaceWidth(pProp, strNewValue, strOldValue, true);
        }

        else if (sName == pfnStartPtBottomElev)
        {
            WriteStartPtBottomElev(pProp, strNewValue, strOldValue);
        }

        else if (sName == pfnEndPtBottomElev)
        {
            WriteEndPtBottomElev(pProp, strNewValue, strOldValue);
        }
        else
        {
            GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
        }
    }
    else
    {
        if (sName == pfnLeftSlopeFactor)
        {
            WriteLeftSlopeFactor(pProp, strNewValue, strOldValue);
        }
        else if (sName == pfnRightSlopeFactor)
        {
            WriteRightSlopeFactor(pProp, strNewValue, strOldValue);
        }

        else if (sName == pfnLeftWorkingFaceWidth || sName == pfnRightWorkingFaceWidth)
        {
            WriteWorkingFaceWidth(pProp, strNewValue, strOldValue);
        }

        else if (sName == pfnStartPtBottomElev)
        {
            WriteStartPtBottomElev(pProp, strNewValue, strOldValue);
        }

        else if (sName == pfnEndPtBottomElev)
        {
            WriteEndPtBottomElev(pProp, strNewValue, strOldValue);
        }
        else
        {
            GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
        }
    }
}

void GCLDitchEarthPropInforWriter::WriteLeftSlopeFactor(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    QString strSetValue = handleValue(strNewValue);
    safeSetProp(pProp->owner()->propByName(pfnLeftSlopeFactor), QString::number(fRound(strSetValue.toDouble(), 3, 0.001), 'f', 3));
}

void GCLDitchEarthPropInforWriter::WriteRightSlopeFactor(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    QString strSetValue = handleValue(strNewValue);
    safeSetProp(pProp->owner()->propByName(pfnRightSlopeFactor), QString::number(fRound(strSetValue.toDouble(), 3, 0.001), 'f', 3));
}

void GCLDitchEarthPropInforWriter::WriteWorkingFaceWidth(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue, bool bIsMirror)
{
    const std::wstring &sName = pProp->propName();
    QString strSetValue = handleInteger(strNewValue);
    if (bIsMirror)
    {
        if (sName == pfnLeftWorkingFaceWidth)
        {
            safeSetProp(pProp->owner()->propByName(pfnRightWorkingFaceWidth), strSetValue);
        }
        else if (sName == pfnRightWorkingFaceWidth)
        {
            safeSetProp(pProp->owner()->propByName(pfnLeftWorkingFaceWidth), strSetValue);
        }
    }
    else
    {
        if (sName == pfnLeftWorkingFaceWidth)
        {
            safeSetProp(pProp->owner()->propByName(pfnLeftWorkingFaceWidth), strSetValue);
        }
        else if (sName == pfnRightWorkingFaceWidth)
        {
            safeSetProp(pProp->owner()->propByName(pfnRightWorkingFaceWidth), strSetValue);
        }
    }
}

void GCLDitchEarthPropInforWriter::WriteStartPtBottomElev(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    if (!strNewValue.trimmed().isEmpty())
    {
        GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
    }
    else
    {
        pProp->initDefaultValue();
    }
}

void GCLDitchEarthPropInforWriter::WriteEndPtBottomElev(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    if (!strNewValue.trimmed().isEmpty())
    {
        GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
    }
    else
    {
        // safeSetProp(pProp->owner()->propByName(pfnEndPtBottomElev), qApp->translate(c_strClassName, c_strFloorBottomElev));
        pProp->initDefaultValue();
    }
}

/*!
*@file
*@brief    处理属性和类别联动问题
*@author   yangwl-a
*@date     2015年4月2日
*/
GTJNameAndTypePropInfoWriter::GTJNameAndTypePropInfoWriter(std::vector<int> * pAcceptEntTypes /*= nullptr*/)
    :GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.push_back(GString::fromStdWString(GTJDBCConsts::pfnType1));
    m_AcceptProps.push_back(GString::fromStdWString(pfnDescription));
    m_AcceptProps.push_back(GString::fromStdWString(pfnGGJType));
}
GTJNameAndTypePropInfoWriter::~GTJNameAndTypePropInfoWriter()
{
}

void GTJNameAndTypePropInfoWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    assert(pProp);
    //土建业务处理
    GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
}

bool GTJNameAndTypePropInfoWriter::validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg)
{
    return GMPPropInfoDefaultWriter::validateProp(pProp, strValue, strErrMsg);
}

GTJAxisOffsetPropInfoWriter::GTJAxisOffsetPropInfoWriter(std::vector<int>* pAcceptEntTypes)
    :GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.push_back(GString::fromStdWString(pfnAxisOffset));
    m_AcceptProps.push_back(GString::fromStdWString(GTJDBCConsts::pfnSideBeamAxisOffset));
}

GTJAxisOffsetPropInfoWriter::~GTJAxisOffsetPropInfoWriter()
{
}

void GTJAxisOffsetPropInfoWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    getEmbedBeam(pProp);
    GString strRealValue = GTJAxisOffsetLineWidthProcessor::calcRealAxisOffsetValue(pProp, strNewValue);
    GMPPropInfoDefaultWriter::writeProp(pProp, strRealValue, strOldValue);
    //在GCLWallPropListener中处理
    //updateAllWallConnectionInfo(pProp);
    updateEmbedBeamThroughWall(pProp, strNewValue, strOldValue);
}

void GTJAxisOffsetPropInfoWriter::updateAllWallConnectionInfo(GMPPropPtr pProp)
{
    IGMPProperties *const pPropList = pProp->owner();
    IGMPEObject *const pObject = pPropList->owner();
    auto const nElmType = pObject->elementType();
    if ((etWall == nElmType || etBrickWall == nElmType || gtj::etCurtainWall == nElmType) 
        && (pObject != nullptr) && (pObject->type() == IGMPEObject::eoEDO) && (pfnAxisOffset == pProp->propName()))
    {
        IGMPElementDrawObj *pEdo = dynamic_cast<IGMPElementDrawObj *>(pObject);
        if (pEdo == nullptr)
            return;

        GTJWallConnection4EditCmd* pWallConnect = new GTJWallConnection4EditCmd(pEdo->contnr()->model());
        pWallConnect->addConnectionWallEdo(pEdo);
        IGMPRelationOperator* pOr = pEdo->contnr()->model()->oprCenter()->relaOpr();
        IGMPElementDrawObj* pEdoRela = pOr->first(pEdo);
        while (pEdoRela != nullptr)
        {
            pWallConnect->disableConnectionInfo(pEdoRela);
            pEdoRela = pOr->next(pEdoRela);
        }

        pWallConnect->enableConnectionInfo();
        delete pWallConnect;
    }
}

bool GTJAxisOffsetPropInfoWriter::validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg)
{
    GString strRealValue = GTJAxisOffsetLineWidthProcessor::calcRealAxisOffsetValue(pProp, strValue);
    return GMPPropInfoDefaultWriter::validateProp(pProp, strRealValue, strErrMsg);
}

void GTJAxisOffsetPropInfoWriter::getEmbedBeam( GMPPropPtr pProp )
{
    if (pProp->propName() != pfnAxisOffset)
    {
        return;
    }
    IGMPEObject *pOwner = pProp->owner()->owner();
    if (pOwner->elementType() != gtj::etForceWall  && pOwner->elementSubType() != 7)
    {
        return;
    }
    IGMPElementDrawObj *pEdo = dynamic_cast<IGMPElementDrawObj*>(pOwner);
    if (pEdo == NULL)
    {
        return;
    }
    m_embedBeamList.clear();
    auto pWallShape = dynamic_cast<IGMPCustomLineSolidShape *>(pEdo->shape());
    CCurve2dPtr pWallCurve = pWallShape->worldLine();
    CBodyPtr pWallBody =  pWallShape->body();
    CBox3d oBox = pWallBody->Box();
    int nWallThickness = pEdo->properties()->asInteger(pfnThickness);

    IGMPElementTypeSet pElementTypeSet;
    pElementTypeSet.insert(etEmbedBeam);
    IGMPEdoIterator* pEdoIter = pEdo->contnr()->createIter(&pElementTypeSet, oBox);
    SCOPE_EXIT
    {
        if (pEdoIter != nullptr)
        {
            delete pEdoIter;
        }
    };
    pEdoIter->first();
    while (!pEdoIter->isDone())
    {
        IGMPElementDrawObj *pEmbedEdo = pEdoIter->curItem();

        /*① 厚度相同*/
        int nEmbedWidth = pEmbedEdo->properties()->asInteger(pfnSectionWidthExt);
        if (nWallThickness != nEmbedWidth)
        {
            pEdoIter->next();
            continue;
        }

        /*② 线类型一致*/
        auto pEmbedShape = dynamic_cast<IGMPCustomLineSolidShape *>(pEmbedEdo->shape());
        CCurve2dPtr pEmbedCurve = pEmbedShape->worldLine();
        if (pWallCurve->Type() != pEmbedCurve->Type())
        {
            pEdoIter->next();
            continue;
        }
        bool bIsSameDir = false;
        /*③ 直线平行*/
        if (pWallCurve->Type() == Line2dType)
        {
            const double c_dParallelEpsilon = 2E-3;  //平行的最大弧度
            if (!GMPPositionFunc2d::isParallel(pWallCurve.get(), pEmbedCurve.get(), c_dParallelEpsilon, c_dParallelEpsilon))
            {
                pEdoIter->next();
                continue;
            }
            if ((dynamic_cast<CLine2d *>(pWallCurve.get())->Dir()).IsEqual(dynamic_cast<CLine2d *>(pEmbedCurve.get())->Dir()))
            {
                bIsSameDir = true;
            }
        }
        else if(pWallCurve->Type() == Arc2dType)
        {
            /*④ 弧线圆心,半径相同*/
            CArc2d *pWallArc2d = dynamic_cast<CArc2d *>(pWallCurve.get());
            CArc2d *pEmbedArc2d = dynamic_cast<CArc2d *>(pEmbedCurve.get());
            CVector2d vWallVec = pWallArc2d->CenterPt();
            CVector2d vEmbedVec = pEmbedArc2d->CenterPt();
            if(!vWallVec.IsEqual(vEmbedVec) || !ggp::sameValue(pWallArc2d->Radius(), pEmbedArc2d->Radius(), ggp::g_DistEpsilon))
            {
                pEdoIter->next();
                continue;
            }
            if (pWallArc2d->ClockSign() == pEmbedArc2d->ClockSign())
            {
                bIsSameDir = true;
            }
        }

        /*⑤ body相交*/
        CBodyPtr pEmbedBody = pEmbedShape->body()/*->Clone()*/;
        BodyPosition nRet = CPositionJudge().GetBodyPosition(pEmbedBody.get(), pWallBody.get());
        if (nRet != bpIntersection)
        {
            pEdoIter->next();
            continue;
        }
        m_embedBeamList.push_back(make_pair(pEmbedEdo, bIsSameDir));
        pEdoIter->next();
    }


}

void GTJAxisOffsetPropInfoWriter::updateEmbedBeamThroughWall(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    if (m_embedBeamList.empty())
    {
        return;
    }
    IGMPElementDrawObj *pWallEdo = dynamic_cast<IGMPElementDrawObj *>(pProp->owner()->owner());
    IGMPCustomLineSolidShape *pWallShape = dynamic_cast<IGMPCustomLineSolidShape *>(pWallEdo->shape());
    bool bIsEmpty = strNewValue.isEmpty();
    double doffset = -1;
    if (!bIsEmpty)
    {
        doffset = strNewValue.toDouble();
    }
    for (auto iter = m_embedBeamList.begin(); iter != m_embedBeamList.end(); ++iter)
    {
        IGMPElementDrawObj *pEmbedEdo = iter->first;
        bool bIsSameDir = iter->second;
        if (bIsEmpty)
        {
            pEmbedEdo->properties()->propByName(pfnAxisOffset)->setIsNull();
        }
        else
        {
            auto pEmbedShape = dynamic_cast<IGMPCustomLineSolidShape *>(pEmbedEdo->shape());
            bool bReveres = false;
            if (!bIsSameDir)
            {
                bReveres = !bReveres;
            }
            //如果暗梁是镜像的，左边周距离取反
            if (pEmbedShape->isMirror())
            {
                bReveres = !bReveres;
            }
            //如果墙是镜像的，左边周距离取反
            if (pWallShape->isMirror())
            {
                bReveres = !bReveres;
            }
            if (bReveres)
            {
                int nWidth = pEmbedEdo->properties()->asInteger(pfnSectionWidthExt);
                doffset = nWidth - doffset;
            }
            pEmbedEdo->properties()->propByName(pfnAxisOffset)->setAsDouble(doffset);
        }

    }
}


GTJLineWidthPropInfoWriter::GTJLineWidthPropInfoWriter(std::vector<int> *const pAcceptEntTypes)
    : GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.push_back(GString::fromStdWString(pfnThickness));
    m_AcceptProps.push_back(GString::fromStdWString(pfnSectionWidth));
    //m_AcceptProps.push_back(GString::fromStdWString(pfnSectionWidth_JZBZ));
    m_AcceptProps.push_back(GString::fromStdWString(pfnWidth));
    m_AcceptProps.push_back(GString::fromStdWString(pfnSectionRadius));
    m_AcceptProps.push_back(GString::fromStdWString(pfnBottomWidth));
    m_AcceptProps.push_back(GString::fromStdWString(pfnFrameThickness));
}

GTJLineWidthPropInfoWriter::~GTJLineWidthPropInfoWriter()
{
    // null sentence
}

double GTJLineWidthPropInfoWriter::getCurrentValue(const QString &oValue, GMPPropPtr const pProp)
{
    bool bStatus;
    double dValue = oValue.toDouble(&bStatus);
    if (bStatus)
    {
        return dValue;
    }
    dValue = pProp->schema()->defaultExpr().toDouble(&bStatus);
    return (bStatus ? dValue : 0);
}

void GTJLineWidthPropInfoWriter::writeProp(GMPPropPtr pProp, const GString &strNewValue, const GString &strOldValue)
{
    //added by zhangh-t 一些构建类型的属性没有必要做下面复杂的关联
    if (defaultWrite(pProp, strNewValue, strOldValue))
    {
        return;
    }

    IGMPProperties *const pPropList = pProp->owner();
    IGMPEObject *const pObject = pPropList->owner();
    auto const nElmType = pObject->elementType();
    if ((etBeam == nElmType) && (pObject->type() == IGMPEObject::eoENT) && (pProp->propName().compare(pfnSectionWidth) == 0))
    {
        return;
    }

    //由于镜像后的轴线距左边线距离显示的值是计算出的真实值，所以修改厚度会导致轴线距左边线距离变化，所以这里需要先算一下，写进去；
    if (pPropList->hasProp(pfnAxisOffset))
    {
        if (pProp->schema()->publicFlag())
        {
            IGMPElement *pElement;
            if (pObject->type() == IGMPEObject::eoENT)
            {
                pElement = dynamic_cast<IGMPElement *>(pObject);
            }
            else
            {
                pElement = dynamic_cast<IGMPElementDrawObj *>(pObject)->element();
            }
            if (pElement != nullptr)
            {
                bool bRet = false;
                IGMPCustomLineSolidShape* pLineShape = nullptr;
                IGMPElementDrawObj * pTmpEdo = nullptr;
                const std::unique_ptr<IGMPEdoIterator> pEdoIter(pElement->createEdoIter());
                //zhangh-t fixed bug TJGJ-29533 带形窗特殊处理
                const bool bIsRibbonWin = (nElmType == etRibbonWin);
                const bool bIsBeam = (nElmType == etBeam || nElmType == etFDBeam);
                for (pEdoIter->first(); !pEdoIter->isDone(); pEdoIter->next())
                {
                    pTmpEdo = pEdoIter->curItem();
                    pLineShape = dynamic_cast<IGMPCustomLineSolidShape*>(pTmpEdo->shape());
                    const bool bDataNull = pTmpEdo->properties()->isDataNull(
                        bIsBeam ? pfnAxisOffset_JZBZ : pfnAxisOffset);

                    if ((nullptr != pLineShape) && !bDataNull)
                    {
                        try
                        {
                            double dWidth = getCurrentValue(strNewValue, pProp);
                            double dNewAxisOffset = pTmpEdo->properties()->asDouble(bIsBeam ? pfnAxisOffset_JZBZ : pfnAxisOffset);
                            if (!bIsRibbonWin && IsLessThan(dWidth, dNewAxisOffset, g_DistEpsilon))
                            {
                                // 如果是梁，则处理pfnAxisOffset_JZBZ
                                pTmpEdo->properties()->setIsNull(bIsBeam ? pfnAxisOffset_JZBZ : pfnAxisOffset);
                                pTmpEdo->contnr()->model()->calculate();
                            }
                            GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
                            pTmpEdo->contnr()->model()->calculate();
                        }
                        catch (const GMPModelException &)
                        {
                            GMPPropInfoDefaultWriter::writeProp(pProp, GString::number(INT_MAX), strOldValue);
                        }
                        bRet = true;
                    }
                }

                if (!bIsRibbonWin &&
                    (strNewValue.isEmpty()
                    || IsLessThan(strNewValue.toDouble(), pElement->properties()->asDouble(pfnAxisOffset), g_DistEpsilon)))
                {
                    // 如果是梁，则处理pfnAxisOffset_JZBZ
                    pElement->properties()->setIsNull(bIsBeam ? pfnAxisOffset_JZBZ : pfnAxisOffset);
                }
                if (bRet)
                {
                    changeLintelAxisOffset(pProp);  
                    //updateAllWallConnectionInfo(pProp);
                    return;
                }
            }
        }
        else
        {
            IGMPElementDrawObj *const pEDO = dynamic_cast<IGMPElementDrawObj *>(pObject);
            if (pEDO != nullptr)
            {
                bool bRet = false;
                IGMPCustomLineSolidShape* pLineShape = dynamic_cast<IGMPCustomLineSolidShape*>(pEDO->shape());
                const bool bIsBeam = (nElmType == etBeam || nElmType == etFDBeam);
                if ((nullptr != pLineShape) && (pLineShape->isMirror()))
                {
                    double dWidth = getCurrentValue(strNewValue, pProp);
                    double dNewAxisOffset = pEDO->properties()->asDouble(bIsBeam ? pfnAxisOffset_JZBZ : pfnAxisOffset);
                    if (IsLessThan(dWidth, dNewAxisOffset, g_DistEpsilon))
                    {
                        // 如果是梁，则处理pfnAxisOffset_JZBZ
                        pEDO->properties()->setIsNull(bIsBeam ? pfnAxisOffset_JZBZ : pfnAxisOffset);
                        pEDO->contnr()->model()->calculate();
                    }

                    GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
                    pEDO->contnr()->model()->calculate();
                    bRet = true;
                }

                if (bRet)
                {
                    changeLintelAxisOffset(pProp);
                    //在GCLWallPropertyListener中处理
                    //updateAllWallConnectionInfo(pProp);
                    return;
                }
            }
        }
    }

    resumeEdoDefaultValue(pProp, strNewValue);
    if (pObject->type() == IGMPEObject::eoEDO)
    {
        writeEdoProp(pProp, strNewValue, strOldValue);
        //updateAllWallConnectionInfo(pProp);
        return;
    }

    //写入属性
    GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
    if (pObject->type() == IGMPEObject::eoENT)
    {
        if (!isNeedChangeAxialOffset(nElmType))
        {
            changeLintelAxisOffset(pProp);
            return;
        }
        //处理构建属性联动
        changeLintelAxisOffset(pProp);
        writeElementProp(pProp, strNewValue, strOldValue);
        //updateAllWallConnectionInfo(pProp);
        return;
    }
}

void GTJLineWidthPropInfoWriter::updateAllWallConnectionInfo(GMPPropPtr pProp)
{
    IGMPProperties *const pPropList = pProp->owner();
    IGMPEObject *const pObject = pPropList->owner();
    auto const nElmType = pObject->elementType();
    if ((etWall == nElmType || etBrickWall == nElmType || gtj::etCurtainWall == nElmType) && (pfnThickness == pProp->propName()))
    {
        IGMPElement *pElement = nullptr;
        if (pObject->type() == IGMPEObject::eoENT)
            pElement = dynamic_cast<IGMPElement *>(pObject);
        else
            pElement = dynamic_cast<IGMPElementDrawObj *>(pObject)->element();

        if (pElement == nullptr)
            return;

        GTJWallConnection4EditCmd* pWallConnect = new GTJWallConnection4EditCmd(pElement->contnr()->model());
        const std::unique_ptr<IGMPEdoIterator> pEdoIter(pElement->createEdoIter());
        IGMPElementDrawObj * pTmpEdo = nullptr;
        for (pEdoIter->first(); !pEdoIter->isDone(); pEdoIter->next())
        {
            pTmpEdo = pEdoIter->curItem();
            if (pTmpEdo != nullptr)
                pWallConnect->disableConnectionInfo(pTmpEdo);
        }
        pWallConnect->enableConnectionInfo();
        delete pWallConnect;
    }
}

void GTJLineWidthPropInfoWriter::writeElementProp(GMPPropPtr pProp, const GString &strNewValue,
    const GString &strOldValue)
{
    double dLineWidth = 0.0;
    //针对存在此属性的构建进行处理
    if (!isChangeElementProp(pProp, dLineWidth))
    {
        return;
    }
    IGMPProperties *const pPropList = pProp->owner();
    if (pPropList->hasProp(pfnAxisOffset))
    {
        IGMPElement *const pElement = dynamic_cast<IGMPElement*>(pPropList->owner());
        if (pElement != nullptr)
        {
            /*pElement->contnr()->model()->calculate();*/
            if (!pPropList->isDataNull(pfnAxisOffset))
            {
                double dLeftWidth = pPropList->asDouble(pfnAxisOffset);
                if (ggp::IsGreaterThan(dLeftWidth, dLineWidth, ggp::g_DistEpsilon))
                {
                    pPropList->setIsNull(pfnAxisOffset);
                    if (pPropList->hasProp(pfnAxisOffset_JZBZ))
                    { //梁钢筋业务属性
                        pPropList->setIsNull(pfnAxisOffset_JZBZ);
                    }
                }
            }

            const std::unique_ptr<IGMPEdoIterator> itrEdo(pElement->createEdoIter());
            for (itrEdo->first(); !itrEdo->isDone(); itrEdo->next())
            {
                IGMPElementDrawObj *const pEdo = itrEdo->curItem();
                IGMPCustomLineSolidShape *const pShape = dynamic_cast<IGMPCustomLineSolidShape*>(pEdo->shape());
                if (pShape != nullptr)
                {
                    //modified by zhangh-t
                    double dLeftWidth = pShape->leftWidth(false);
                    if (ggp::IsGreaterThan(dLeftWidth, dLineWidth, ggp::g_DistEpsilon))
                    {
                        pEdo->properties()->setIsNull(pfnAxisOffset);
                        if (pPropList->hasProp(pfnAxisOffset_JZBZ))
                        { //梁钢筋业务属性
                            pPropList->setIsNull(pfnAxisOffset_JZBZ);
                        }
                    }
                }
            }
        }
    }

}

void GTJLineWidthPropInfoWriter::writeEdoProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    IGMPElementDrawObj *const pEDO = dynamic_cast<IGMPElementDrawObj*>(pProp->owner()->owner());
    if (pEDO != nullptr)
    {
        //处理板带图元宽度与轴线距板带左边线距离联动
        GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
        pEDO->contnr()->model()->calculate();

        IGMPProperties *const pPropList = pProp->owner();
        //处理过梁属性的联动
        changeLintelAxisOffset(pProp);
        //处理板带暗梁联动
        if (pPropList->hasProp(GTJDBCConsts::pfnSideBeamAxisOffset))
        {
            const bool bDataNull = pEDO->properties()->isDataNull(GTJDBCConsts::pfnSideBeamAxisOffset);
            if (!bDataNull)
            {
                double dWidth = getCurrentValue(strNewValue, pProp);
                double dNewAxisOffset = pEDO->properties()->asDouble(GTJDBCConsts::pfnSideBeamAxisOffset);
                if (IsLessThan(dWidth, dNewAxisOffset, g_DistEpsilon))
                {
                    pEDO->properties()->setIsNull(GTJDBCConsts::pfnSideBeamAxisOffset);
                    pEDO->contnr()->model()->calculate();
                }
            }
        }

    }
}

bool GTJLineWidthPropInfoWriter::isChangeElementProp(GMPPropPtr pProp, double &dWidth)
{
    do
    {
        IGMPProperties *const pProps = pProp->owner();

        if (pProps->hasProp(pfnWidth))
        {
            dWidth = pProps->asDouble(pfnWidth);
            break;
        }

        if (pProps->hasProp(pfnThickness))
        {
            dWidth = pProps->asDouble(pfnThickness);
            break;
        }
        else if (pProps->hasProp(pfnSectionWidth))
        {
            dWidth = pProps->asDouble(pfnSectionWidth);
            break;
        }
        else if (pProps->hasProp(pfnBottomWidth))
        {
            dWidth = pProps->asDouble(pfnBottomWidth);
            break;
        }
        else if (pProps->hasProp(pfnFrameThickness))
        {
            dWidth = pProps->asDouble(pfnFrameThickness);
            break;
        }
        else if (pProps->hasProp(pfnSectionRadius))
        {
            dWidth = pProps->asDouble(pfnSectionRadius);
            break;
        }
        return false;
    } while (false);
    return true;
}

void GTJLineWidthPropInfoWriter::resumeEdoDefaultValue(GMPPropPtr pProp, const GString &strNewValue)
{
    if ((pProp->owner()->owner()->type() == IGMPEObject::eoEDO) && (pProp->propName() == pfnThickness))
    {
        const GString oValue = strNewValue.trimmed();
        if (isNullValue(pProp, oValue) || canResumeDefaultValue(pProp, oValue))
        {
            pProp->initDefaultValue();
        }
    }
}

bool GTJLineWidthPropInfoWriter::isNeedChangeAxialOffset(const int nElementType)
{
    switch (nElementType)
    {
        case etRibbonWin:
            return false;
        default:
            return true;
    }
}

void GTJLineWidthPropInfoWriter::changeLintelAxisOffset(GMPPropPtr pProp)
{
    IGMPProperties *const pPropList = pProp->owner();
    IGMPEObject *const pObject = pPropList->owner();
    //公有属性取当前构件的图元进行逐一判断
    if (pProp->schema()->publicFlag())
    {
        IGMPElement* pElement = NULL;
        if (pObject->type() == IGMPEObject::eoENT)
        {
            pElement = dynamic_cast<IGMPElement*>(pObject);
        }
        else
        {
            pElement = dynamic_cast<IGMPElementDrawObj*>(pObject)->element();
        }
        if (pElement != NULL)
        {
            pElement->contnr()->model()->beginEdit();
            SCOPE_EXIT
            {
                pElement->contnr()->model()->endEdit();
            };
            const std::unique_ptr<IGMPEdoIterator> pEdoIterPtr(pElement->createEdoIter());
            IGMPElementDrawObj* pTempEdo = NULL;
            for (pEdoIterPtr->first(); !pEdoIterPtr->isDone(); pEdoIterPtr->next())
            {
                pTempEdo = pEdoIterPtr->curItem();
                changeValideLintelAxisOffset(pTempEdo);
            }
        }
    }
    //私有属性取当前图元进行判断
    else
    {
        IGMPElementDrawObj *const pEDO = dynamic_cast<IGMPElementDrawObj *>(pObject);
        if (pEDO != nullptr)
        {
            pEDO->contnr()->model()->beginEdit();
            SCOPE_EXIT
            {
                pEDO->contnr()->model()->endEdit();
            };
            changeValideLintelAxisOffset(pEDO);
        }
    }
}
void GTJLineWidthPropInfoWriter::changeValideLintelAxisOffset(IGMPElementDrawObj *pTempEdo)
{
    IGMPRelationOperator* pRela = pTempEdo->contnr()->model()->oprCenter()->relaOpr();
    const int nChildCount = pRela->childrenCount(pTempEdo);
    //对修改带型窗框厚的处理
    if (pTempEdo->elementType() == etRibbonWin)
    {
        for (int i = 0; i < nChildCount; ++i)
        {
            IGMPElementDrawObj *pLintelEdo = pRela->child(pTempEdo, i);
            if (pLintelEdo->elementType() == etLintel)
            {
                setLintelAxisOffset(pLintelEdo, pTempEdo);
            }
        }
    }
    else
    {
        //对其他类似修改墙的厚度的处理
        for (int i = 0; i < nChildCount; ++i)
        {
            IGMPElementDrawObj* pDoorEdo = pRela->child(pTempEdo, i);
            const int nElement = pDoorEdo->elementType();
            switch (nElement)
            {
                case etDoor:
                case etWindow:
                case etOpening:
                case etAlcove:
                case etDoorWin:
                {
                    IGMPRelationOperator* pRela1 = pDoorEdo->contnr()->model()->oprCenter()->relaOpr();
                    const int nDoorChild = pRela1->childrenCount(pDoorEdo);
                    for (int j = 0; j < nDoorChild; ++j)
                    {
                        IGMPElementDrawObj *pLintelEdo = pRela1->child(pDoorEdo, j);
                        if (pLintelEdo->elementType() == etLintel)
                        {
                            setLintelAxisOffset(pLintelEdo, pDoorEdo);
                        }
                    }
                    break;
                }
                default:
                    break;
            }
        }
    }
}
void GTJLineWidthPropInfoWriter::setLintelAxisOffset(IGMPElementDrawObj * pLintelEdo, IGMPElementDrawObj* pParentEdo)
{
    IGMPSectionLineSolidShape* pTempShape = dynamic_cast<IGMPSectionLineSolidShape*>(pLintelEdo->shape());
    const double dWidth = pTempShape->lineWidth();
    const double dAxisOffset = pLintelEdo->properties()->asDouble(pfnAxisOffset);
    const double dWallWidth = getLintelWallWidth(pParentEdo);
    if ((vrLessThan != ggp::compareValue(dAxisOffset, dWidth / 2 + dWallWidth, g_DistEpsilon))
        || (vrGreaterThan != ggp::compareValue(dAxisOffset, -dWidth / 2, g_DistEpsilon)))
    {
        pLintelEdo->properties()->propByName(pfnAxisOffset)->setIsNull();
    }
}

bool GTJLineWidthPropInfoWriter::defaultWrite(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    if (pProp->owner()->owner()->elementType() == etBedding && pProp->propName() == pfnThickness)
    {
        //垫层的厚度属性和偏心距没有关系
        GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
        return true;
    }
    return false;
}

GString GTJAxisOffsetLineWidthProcessor::calcRealAxisOffsetValue(GMPPropPtr pProp, const GString& strValue)
{
    auto const pPropList = pProp->owner();
    auto const pObject = pPropList->owner();
    if (pObject->type() == IGMPEObject::eoEDO)
    {
        bool bStatus;
        auto const nValue = strValue.toInt(&bStatus);
        if (bStatus)
        {
            IGMPElementDrawObj *const pEDO = dynamic_cast<IGMPElementDrawObj*>(pObject);
            if (pProp->propName().compare(pfnAxisOffset_JZBZ) == 0)
            {
                if (pPropList->hasProp(pfnSectionWidth_JZBZ))
                {
                    IGMPCustomLineSolidShape *const pShape = dynamic_cast<IGMPCustomLineSolidShape *>(pEDO->shape());
                    if (nullptr != pShape && pShape->isMirror())
                    {
                        int nSectionWidth = pEDO->properties()->asInteger(pfnSectionWidth_JZBZ);
                        return QString::number(nSectionWidth - nValue);
                    }
                }
            }
            else
            {
                IGMPCustomLineSolidShape *const pShape = dynamic_cast<IGMPCustomLineSolidShape *>(pEDO->shape());
                if (nullptr != pShape && pShape->isMirror())
                {
                    return QString::number(pShape->lineWidth() - nValue);
                }
            }
        }
    }
    return strValue;
}

QString calcRealAxisOffset(GMPPropPtr const pProp, const QString &strValue)
{
    if (pProp != nullptr)
    {
        auto const pObject = pProp->owner()->owner();
        if (pObject->type() == IGMPEObject::eoEDO)
        {
            auto const pShape = dynamic_cast<IGMPElementDrawObj *>(pObject)->shape();
            if (pShape->shapeClass() == scLine)
            {
                auto const pLineShape = dynamic_cast<IGMPCustomLineSolidShape*>(pShape);
                if (pLineShape->isMirror())
                {
                    bool bStatus;
                    const double dValue = strValue.toDouble(&bStatus);
                    if (bStatus)
                    {
                        return QString::number(pLineShape->lineWidth() - dValue);
                    }
                }
            }
        }
    }
    return strValue;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
GTJVariSecPropInfoWriter::GTJVariSecPropInfoWriter(vector<int> *const pAcceptEntTypes)
    : GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.clear();
    m_AcceptProps.push_back(GString::fromStdWString(pfnSectionHeight));
    m_AcceptProps.push_back(GString::fromStdWString(pfnSectionWidth));
    m_AcceptProps.push_back(GString::fromStdWString(pfnSectionRadius));
}

void GTJVariSecPropInfoWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    GMPPropPtr pPropTop = nullptr;
    auto const pPropName = pProp->propName();
    if (pPropName == pfnSectionWidth)
    {
        pPropTop = pProp->owner()->propByName(pfnTopSectionWidth);
    }
    else if (pPropName == pfnSectionHeight)
    {
        pPropTop = pProp->owner()->propByName(pfnTopSectionHeight);
    }
    else if (pPropName == pfnSectionRadius)
    {
        pPropTop = pProp->owner()->propByName(pfnTopSectionRadius);
    }
    GString oValue = strNewValue.trimmed();
    if (oValue.isEmpty())
    {
        if (canResumeDefaultValue(pProp, oValue))
        {
            pProp->initDefaultValue();
        }
        else if (isNullValue(pProp, oValue))
        {
            pProp->setIsNull();
        }

        if (pPropTop != nullptr)
        {
            if (canResumeDefaultValue(pPropTop, oValue))
            {
                pPropTop->initDefaultValue();
            }
            else if (isNullValue(pPropTop, oValue))
            {
                pPropTop->setIsNull();
            }
        }
    }
    else
    {
        const QStringList oValueList = oValue.split('/', QString::SkipEmptyParts);
        pProp->setAsDouble(oValueList.first().toDouble());
        if (pPropTop != nullptr)
        {
            pPropTop->setAsDouble(oValueList.last().toDouble());
        }
    }

    if (!validateBody(pProp))
    {
        GlodonMessageBox::information(QApplication::activeWindow(),
            qApp->translate("GTJPropInfoWriter", s_strTips),
            qApp->translate("GTJPropInfoWriter", c_strBodyIntersected), GlodonMessageBox::Ok);
        writeProp(pProp, strOldValue, strOldValue);
    }
}

bool GTJVariSecPropInfoWriter::validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg)
{
    if (canResumeDefaultValue(pProp, strValue) || isNullValue(pProp, strValue))
    {
        return true;
    }
    QStringList oValueList = strValue.trimmed().split('/', QString::SkipEmptyParts);
    if (oValueList.count() > 2 || oValueList.isEmpty())
    {
        strErrMsg = qApp->translate(c_pGTJPropInfoWriter, c_strSectionWriterType);
        return false;
    }
    // 对属性值进行单位化
    for (auto itr = oValueList.begin(), constEnd = oValueList.end(); itr != constEnd; ++itr)
    {
        *itr = GTJPropCommon::unitTransferElev(pProp, *itr, m_pUnitTransfer);
        try
        {
            pProp->check(*itr);
        }
        catch (const GMPModelException &except)
        {
            strErrMsg = except.message();
            return false;
        }
    }
    return true;
}

bool GTJVariSecPropInfoWriter::validateBody(GMPPropPtr const pProp)
{
    auto pObject = pProp->owner()->owner();
    if (pObject->type() == IGMPEObject::eoENT)
    {
        IGMPElementDrawObj *pEdo;
        IGMPElement *const pElement = dynamic_cast<IGMPElement *>(pObject);
        const std::unique_ptr<IGMPEdoIterator> pContainer(pElement->createEdoIter());
        for (pContainer->first(); !pContainer->isDone(); pContainer->next())
        {
            pEdo = pContainer->curItem();
            pEdo->contnr()->model()->calculate();
            auto const pShape = dynamic_cast<IGMPPointSolidShape *>(pEdo->shape());
            if (pShape != nullptr)
            {
                auto const pBody = pShape->body();
                if (pBody == nullptr || pBody->IsEmpty() || !pBody->IsStrictlyValid())
                {
                    return false;
                }
            }
        }
    }
    else if (pObject->type() == IGMPEObject::eoEDO)
    {
        IGMPElementDrawObj *const pEdo = dynamic_cast<IGMPElementDrawObj *>(pObject);
        pEdo->contnr()->model()->calculate();
        auto const pShape = dynamic_cast<IGMPPointSolidShape *>(pEdo->shape());
        if (pShape != nullptr)
        {
            auto const pBody = pShape->body();
            return (pBody != nullptr && !pBody->IsEmpty() && pBody->IsStrictlyValid());
        }
        else
        {
            return false;
        }
    }
    return true;
}

/*!
*@brief     土建/钢筋点绘连梁与墙厚度联动
*@author    liuk-g 2015年08月04日
*@return    void
*/
void GTJWallThicknessListener::onNotify(int iNotifyType, IGMPProperties* pProps, void* pData, bool bRedoUndo)
{
    if (pProps == nullptr)
        return;

    IGMPEObject *pObject = pProps->owner();
    if (pObject == nullptr)
        return;
    if (pObject->elementType() != gtj::etForceWall && pObject->elementType() != gtj::etBrickWall)
    {
        return;
    }
    IGMPPropertySchema *pSchema = static_cast<IGMPPropertySchema*>(pData);
	if (pSchema == nullptr)
	{
		return;
	}
    bool isPublic = pSchema->publicFlag();
    /*
    在土建版本中，厚度属性Thickness为公有属性，而在钢筋版本中，其为私有属性
    因此,获得所有连梁图元的方式不同，处理也就不同.
    */
    if (!isPublic)
    {
        //2016-12-1 此时的剪力墙/砌体墙的厚度/偏心距属性均为共有
        return onNotifyByGGJ(iNotifyType, pProps, pData, bRedoUndo);
    }
    if (pSchema->propName() != pfnThickness)
    {
        return;
    }
    IGMPElement *pElement = nullptr;
    //公有属性，通过构件获得所有图元
    if (pObject->type() == IGMPEObject::eoENT)
    {
        pElement = dynamic_cast<IGMPElement *>(pObject);
    }
    else
    {
        IGMPElementDrawObj *pEdo = dynamic_cast<IGMPElementDrawObj *>(pObject);
        pElement = pEdo->element();
    }
    storeCorrelativeObjects(iNotifyType, pElement, static_cast<IGMPPropertySchema*>(pData), bRedoUndo);
}

void GTJWallThicknessListener::onNotify(int iNotifyType, IGMPElement* pEnt, void* pData, bool bUndoRedo)
{
    if (iNotifyType == ntBeforeDelete && pEnt->elementType() == gtj::etForceWall
        || pEnt->elementType() == gtj::etBrickWall)
    {
        auto iter = m_mapElementModified.find(pEnt);
        if (iter != m_mapElementModified.end())
        {
            m_mapElementModified.erase(iter);
        }
    }
}

void GTJWallThicknessListener::onNotify(int iNotifyType, IGMPElementDrawObj* pEdo, void* pData, bool bUndoRedo)
{
    if (iNotifyType == ntBeforeDelete && pEdo->elementType() == gtj::etForceWall
        || pEdo->elementType() == gtj::etBrickWall)
    {
        auto iter = m_mapEDOModified.find(pEdo);
        if (iter != m_mapEDOModified.end())
        {
            m_mapEDOModified.erase(iter);
        }
    }
}

void GTJWallThicknessListener::onNotifyByGGJ(int iNotifyType, IGMPProperties* pProps, void* pData, bool bRedoUndo)
{
    IGMPEObject *pOwner = pProps->owner();
    if (pOwner->type() != IGMPEObject::eoEDO)
        return;
    //处理暗梁距左边线距离，PS连梁不知道
    if (pOwner->elementType() == etSideBeam || pOwner->elementType() == etLinkBeam)
    {
        IGMPElementDrawObj *pEdo = dynamic_cast<IGMPElementDrawObj*>(pOwner);
        if (pEdo == NULL)
        {
            return;
        }
        if (iNotifyType == ntAfterUpdate)
        {
            IGMPPropertySchema *pSchema = reinterpret_cast<IGMPPropertySchema*>(pData);
            if (pSchema->propName() == pfnSectionWidth)
            {
                double dAxisOffset = pEdo->properties()->asDouble(pfnAxisOffset);
                double dSectionWidth = pEdo->properties()->asDouble(pfnSectionWidth);
                if (ggp::compareValue(dSectionWidth, dAxisOffset, g_DistEpsilon) == vrLessThan)
                {
                    pEdo->properties()->setIsNull(pfnAxisOffset);
                }
            }
        }
    }
    //处理墙和梁之间联动
    if (pOwner->elementType() == gtj::etForceWall || pOwner->elementType() == gtj::etBrickWall)
    {
        IGMPElementDrawObj *pEdo = dynamic_cast<IGMPElementDrawObj*>(pOwner);
        IGMPPropertySchema* pPropSchema = static_cast<IGMPPropertySchema*>(pData);
        storeCorrelativeObjects(iNotifyType, pEdo, pPropSchema, bRedoUndo);
    }
}

void GTJWallThicknessListener::onCalculate()
{
    calculateOrEndEdit();
}

void GTJWallThicknessListener::storeCorrelativeObjects(int nNotifyType,
    IGMPEObject* pUpdatedObject, 
    IGMPPropertySchema* pPropSchema,
    bool bUndoRedo)
{
    if (pUpdatedObject == NULL)
    {
        return;
    }
    std::wstring sPropName = pPropSchema->propName();
    if (nNotifyType == ntBeforeUpdate && !bUndoRedo)
    {
        if (sPropName == pfnThickness)
        {
            m_iSectionWith = pUpdatedObject->properties()->asInteger(pfnThickness);
        }
    }
    if (nNotifyType == ntAfterUpdate)
    {
        //只关注这俩属性的变化
        if (sPropName != pfnThickness && sPropName != pfnAxisOffset)
        {
            return;
        }
        if (pUpdatedObject->type() == IGMPEObject::eoEDO)
        {
            IGMPElementDrawObj* pEDO = dynamic_cast<IGMPElementDrawObj*>(pUpdatedObject);
            m_mapEDOModified.insert(std::make_pair(pEDO, sPropName));
        }
        else
        {
            IGMPElement* pElement = dynamic_cast<IGMPElement*>(pUpdatedObject);
            m_mapElementModified.insert(std::make_pair(pElement, sPropName));
        }
    }
}

void GTJWallThicknessListener::storeCorrelativeObjectsForSingleObject(int nFlag,
    IGMPElementDrawObj* pEDO,
    CPositionJudge* pPositionJudge, 
    std::wstring& sPropName,
    IGMPEdoIterator* pIter)
{
    if (pEDO == nullptr)
    {
        return;
    }
    //修改墙的厚度，需要同步暗梁的厚度
    IGMPEdoIterator* pedoIterator = pIter;
    IGMPRelationOperator* pRelation = pEDO->contnr()->model()->oprCenter()->relaOpr();
    IGMPElementDrawObj* pFirst = pRelation->first(pEDO);
    while (pFirst)
    {
        if (m_setOperatedEDOs.find(pFirst) != m_setOperatedEDOs.end())
        {
            pFirst = pRelation->next(pFirst);
            continue;
        }
        m_setOperatedEDOs.insert(pFirst);
        pedoIterator->first();
        ggp::CBox3d oEDOBox = pEDO->shape()->box();
        while (!pedoIterator->isDone())
        {
            IGMPElementDrawObj *pSideBeamEdo = pedoIterator->curItem();
            if (pSideBeamEdo == nullptr)
            {
                pedoIterator->next();
                continue;
            }
            if (pSideBeamEdo->elementType() == etSideBeam && pSideBeamEdo->properties()->hasProp(pfnGGJType))
            {
                int nType = pSideBeamEdo->properties()->asInteger(pfnGGJType);
                //边框梁不需要联动
                if (nType == 1)
                {
                    pedoIterator->next();
                    continue;
                }
            }
            //楼层是否相同
            if (pSideBeamEdo->floor() != pEDO->floor())
            {
                pedoIterator->next();
                continue;
            }
            //box碰一下
            if (2 == nFlag && !oEDOBox.IsIntersect(pSideBeamEdo->shape()->box()))
            {
                pedoIterator->next();
                continue;
            }
            //判断梁线与墙线是否平行
            IGMPCustomLineSolidShape *pWallShape = dynamic_cast<IGMPCustomLineSolidShape*>(pFirst->shape());
            IGMPCustomLineSolidShape *pSideBeamShape = dynamic_cast<IGMPCustomLineSolidShape*>(pSideBeamEdo->shape());
            CCurve2dPtr pWallLine = pWallShape->worldLine();
            CCurve2dPtr pSideBeamLine = pSideBeamShape->worldLine();
            
            /*解决钢筋精简版测试暗梁系列BUG，联动原则是
            1、长宽一致的梁与墙联动
            2、方向不同，但是长与宽相等也需要联动
            3、多个墙梁，只联动当前所选择的图元
            4、必须考虑镜像之后联动，还有线式绘制联动*/
            EnPolygonPolygonPosition pos = pPositionJudge->GetPolygonPosition(pSideBeamShape->worldPoly().get(), pWallShape->worldPoly().get());
            if (pos == PP_UNKNOWN || pos == PP_SEPARATION || pos == PP_EXTERNAL_TANGENT)
            {
                pedoIterator->next();
                continue;
            }
            if (!isParallel(pWallLine.get(), pSideBeamLine.get(), ggp::g_ParallelEpsilon, ggp::g_DistEpsilon))
            {
                pedoIterator->next();
                continue;
            }
            int nSideBeamWidth = pSideBeamEdo->properties()->asInteger(pfnSectionWidth);
            int nWallThickness = pFirst->properties()->asInteger(pfnThickness);
            NotifyLinkage linkage = { 0 };
            linkage.nLinkageEdoID = pSideBeamEdo->iD();
            linkage.nEdoID = pFirst->iD();
            //判断截面宽度是否一致
            if (sPropName == pfnThickness && nSideBeamWidth == m_iSectionWith)
            {
                linkage.nLinkageType = 0;
                m_vecLinkage.push_back(linkage);
            }
            //处理距左边线距离  modify by liuk-g GTJY-16760 新需求，修改过axisoffset的暗/连梁不再随着墙联动
            if (sPropName == pfnAxisOffset && nSideBeamWidth == nWallThickness && pSideBeamEdo->properties()->isDataNull(sPropName))
            {
                linkage.nLinkageType = 1;
                m_vecLinkage.push_back(linkage);
            }
            pedoIterator->next();
        }
        pFirst = pRelation->next(pFirst);
    }
}

void GTJWallThicknessListener::onEndEdit()
{
    calculateOrEndEdit();
}

void GTJWallThicknessListener::calculateOrEndEdit()
{
    m_setOperatedEDOs.clear();
    CPositionJudge oJudger;
    for (auto iter = m_mapEDOModified.begin(); iter != m_mapEDOModified.end(); ++iter)
    {
        IGMPElementDrawObj* pEDO = iter->first;
        m_pEdoContnr = pEDO->contnr();
        ggp::CBox3d oBox = pEDO->shape()->box();
        IGMPElementTypeSet pElementTypeSet;
        pElementTypeSet.insert(etSideBeam);
        pElementTypeSet.insert(etLinkBeam);
        m_pEdoContnr = pEDO->contnr();
        IGMPEdoIterator* pEDOIter = pEDO->contnr()->createIter(&pElementTypeSet, oBox);
        //内部循环不需要碰撞box
        storeCorrelativeObjectsForSingleObject(1, pEDO, &oJudger, iter->second, pEDOIter);
        delete pEDOIter;
    }
    m_mapEDOModified.clear();

    for (auto iter = m_mapElementModified.begin(); iter != m_mapElementModified.end(); ++iter)
    {
        IGMPElement* pElement = iter->first;
        m_pEdoContnr = pElement->contnr()->model()->edoContnr();
        IGMPEdoIterator* pIter = pElement->createEdoIter(false);
        SCOPE_EXIT{delete pIter;};
        ggp::CBox3d oFilterBox;
        for (pIter->first(); !pIter->isDone(); pIter->next())
        {
            if (oFilterBox.NotEmpty())
            {
                oFilterBox.MergeBox(pIter->curItem()->shape()->box());
            }
            else
            {
                oFilterBox = pIter->curItem()->shape()->box();
            }
        }
        IGMPElementTypeSet pElementTypeSet;
        pElementTypeSet.insert(etSideBeam);
        pElementTypeSet.insert(etLinkBeam);
        IGMPEdoIterator* pEDOIter = pElement->contnr()->model()->edoContnr()->createIter(&pElementTypeSet, oFilterBox);
        for (pIter->first(); !pIter->isDone(); pIter->next())
        {
            //内部循环需要碰撞box来加速
            storeCorrelativeObjectsForSingleObject(2, pIter->curItem(), &oJudger, iter->second, pEDOIter);
        }
        delete pEDOIter;
    }
    m_mapElementModified.clear();
    for (size_t i = 0; i < m_vecLinkage.size(); ++i)
    {
        NotifyLinkage curItem = m_vecLinkage[i];
        IGMPElementDrawObj* pEdo = m_pEdoContnr->find(curItem.nEdoID);
        IGMPElementDrawObj* pSideBeamEdo = m_pEdoContnr->find(curItem.nLinkageEdoID);
        if (!pEdo || !pSideBeamEdo)
        {
            continue;
        }
        //判断梁线与墙线是否平行
        IGMPCustomLineSolidShape *pWallShape = dynamic_cast<IGMPCustomLineSolidShape*>(pEdo->shape());
        IGMPCustomLineSolidShape *pSideBeamShape = dynamic_cast<IGMPCustomLineSolidShape*>(pSideBeamEdo->shape());
        CCurve2dPtr pWallLine = pWallShape->worldLine();
        CCurve2dPtr pSideBeamLine = pSideBeamShape->worldLine();
        int nSideBeamWidth = pSideBeamEdo->properties()->asInteger(pfnSectionWidth);
        int nWallThickness = pEdo->properties()->asInteger(pfnThickness);

        if (curItem.nLinkageType == 0)
        {
            if (pSideBeamShape->isMirror())
            {
                // pSideBeamEdo->properties()->setAsDouble(pfnAxisOffset, nSideBeamWidth);
                pSideBeamEdo->properties()->setAsString(pfnAxisOffset, QString::number(nSideBeamWidth));
            }
            else
            {
                pSideBeamEdo->properties()->setAsString(pfnAxisOffset, QString::number(0));
            }
            pSideBeamEdo->properties()->setAsString(pfnSectionWidth, QString::number(nWallThickness));
            bool bRequire = pEdo->properties()->isDataNull(pfnAxisOffset);
            double doffset = pEdo->properties()->asDouble(pfnAxisOffset);
            if (bRequire || (ggp::compareValue(doffset, nWallThickness, g_DistEpsilon) == vrGreaterThan))
            {
                pSideBeamEdo->properties()->setIsNull(pfnAxisOffset);
            }
            else
            {
                bool bReveres = false;
                //如果不同方向，左边周距离取反
                if (!GMPPositionFunc2d::isSameDirection(pWallLine.get(), pSideBeamLine.get()))
                {
                    bReveres = !bReveres;
                }
                //如果暗梁是镜像的，左边周距离取反
                if (pSideBeamShape->isMirror())
                {
                    bReveres = !bReveres;
                }
                //如果墙是镜像的，左边周距离取反
                if (pWallShape->isMirror())
                {
                    bReveres = !bReveres;
                }
                if (bReveres)
                {
                    doffset = nWallThickness - doffset;
                }
                pSideBeamEdo->properties()->setAsString(pfnAxisOffset, QString::number(doffset));
            }
        }
        else if (curItem.nLinkageType == 1)
        {
            double doffset = pEdo->properties()->asDouble(pfnAxisOffset);
            bool bRequire = pEdo->properties()->isDataNull(pfnAxisOffset);
            if (!bRequire)
            {
                bool bReveres = false;
                //如果不同方向，左边周距离取反
                if (!GMPPositionFunc2d::isSameDirection(pWallLine.get(), pSideBeamLine.get()))
                {
                    bReveres = !bReveres;
                }
                //如果暗梁是镜像的，左边周距离取反
                if (pSideBeamShape->isMirror())
                {
                    bReveres = !bReveres;
                }
                //如果墙是镜像的，左边周距离取反
                if (pWallShape->isMirror())
                {
                    bReveres = !bReveres;
                }
                if (bReveres)
                {
                    doffset = nWallThickness - doffset;
                }
                pSideBeamEdo->properties()->setAsString(pfnAxisOffset, QString::number(doffset));
            }
            else
            {
                pSideBeamEdo->properties()->setIsNull(pfnAxisOffset);
            }
        }
        pEdo->contnr()->model()->calculate();
        pSideBeamEdo->contnr()->model()->calculate();
    }
    m_vecLinkage.clear();
}



/////////////////////////////////////////截面形状三点按钮///////////////////////////////////////////////////////////////////////////////
GTJParamSectionPropInfoWriter::GTJParamSectionPropInfoWriter(std::vector<int> *const pAcceptEntTypes)
    : GTJParamNAbnormalWriter(nullptr, pAcceptEntTypes)
{
    m_AcceptProps.clear();
    m_AcceptProps.push_back(QString::fromStdWString(pfnSectionTypeID));
}

//属性值校验
bool GTJParamSectionPropInfoWriter::validateProp(GMPPropPtr pProp, const GString &strValue, GString &strErrMsg)
{
    try
    {
        pProp->check(strValue);
        return true;
    }
    catch (const GMPModelException &exception)
    {
        strErrMsg = exception.message();
        return false;
    }
    catch (...)
    {
        return false;
    }
}

//截面形状三点按钮点击事件处理
void GTJParamSectionPropInfoWriter::onButtonClick(std::vector<GMPPropPtr> &propVec, bool &modified)
{
    if (propVec.empty())
    {
        return;
    }
    IGMPProperties *pPropList = propVec.front()->owner();
    IGMPEObject *const pObject = pPropList->owner();
    auto const elmType = pObject->elementType();
    if (elmType == -1)
    {
        return;
    }

    IGMPModel *pModel = nullptr;
    if (pObject->type() == IGMPEObject::eoEDO)
    {
        pModel = dynamic_cast<IGMPElementDrawObj *>(pObject)->contnr()->model();
    }
    else if (pObject->type() == IGMPEObject::eoENT)
    {
        pModel = dynamic_cast<IGMPElement *>(pObject)->contnr()->model();
    }
    else
    {
        return;
    }

    const int nSectionTypeID = propVec.front()->asInteger();
    if (esctSectAbnormity == nSectionTypeID)
    { // 异形柱，调用异形截面编辑
        editAbnormitySection(elmType, propVec, modified);
    }
    else if (esctSectParams == nSectionTypeID)
    { // 参数化柱
        ggp::CDatabase *const paramPolyDataBase = pModel->paramPolyDB();
        //new 出来的 Widget 指针， 会被 SelParamPolyForm 接走并负责释放，此处的 new 不需要 delete 对应
        GTJCustomParamWidgetDefault* pWidget = new GTJCustomParamWidgetDefault();
        const std::unique_ptr<GMPParamSectionInfo> pSectionInfo(new GMPParamSectionInfo());
        modified = GTJSelParamPolyIntf::selCommonPoly(paramPolyDataBase, pObject,
            pPropList->asString(pfnPolyValue), pPropList->asInteger(pfnPolyID), pSectionInfo.get(), pWidget);
        if (modified)
        {
            pModel->beginEdit();
            double dWidth = 0.0;
            CVector2d insertPoint(pSectionInfo->strInsPt.section(',', 0, 0).toDouble(),
                pSectionInfo->strInsPt.section(',', -1, -1).toDouble());
            const CPolygonPtr pPolygon = str2Polygon(pSectionInfo->strSectionPoly);
            //判断当前构件是否为参数化柱或构造柱
            auto const elmSubType = pObject->elementSubType();
            const int nParamMasonryColumn = 304;
            const bool parametric = ((elmType == etColumn && elmSubType == 3)
                || (elmType == etTieColumn && elmSubType == 127)
                || (gtj::etMasonryColumn == elmType && nParamMasonryColumn == elmSubType) );
			const bool bPrivateProp = etMasonryReinf == elmType;
            const QString strSectionInfo = paramPolyToString(pSectionInfo->strSectionPoly,
                pSectionInfo->nPolyID, pSectionInfo->strPolyVal, pSectionInfo->strInsPt);
            for (auto itr = propVec.cbegin(), itrEnd = propVec.cend(); itr != itrEnd; ++itr)
            {
                //fix.bug：GTJY-5599 构件公共/修改参数化图元的截面形状中的尺寸，软件崩溃
                pPropList = (*itr)->owner();
                if (parametric) //只有构造柱、柱参数化时才写入
                {
                    pPropList->setAsString(GTJDBCConsts::pfnSectionInfo, strSectionInfo);
                }
                // 设置SectionID
                pPropList->setAsInteger(pfnSectionTypeID, esctSectParams);

                //fixed bug GTJY-5819 这么设置是为了触发数据库变化，在bug描述的场景里构造不出来截面
                pPropList->setAsInteger(pfnPolyID, -1);
                // 设置PolyID
                pPropList->setAsInteger(pfnPolyID, pSectionInfo->nPolyID);
                // 设置插入点
                pPropList->setAsVector2d(pfnInsertPt, insertPoint);
                // 设置参数值
                pPropList->setAsString(pfnPolyValue, pSectionInfo->strPolyVal);
                // 设置SectionPoly
                dWidth = 0;
                if (pPolygon)
                { //fixed bug TJGJ-31176 墙/圆形参数化墙修改截面形状为非法的图元，软件崩溃
                    const CBox2d polyBox = pPolygon->Box();
                    dWidth = polyBox.MaxPt().X - polyBox.MinPt().X;
                    if (_finite(dWidth) && dWidth > 1)
                    {
                        pPolygon->MergeCoedges(); // GTJY-14862 不能使用上面的函数，否则有此bug
                        pPropList->setAsPolygon(pfnSectionPoly, *pPolygon);
                    }
                    else
                    {
                        pPropList->setIsNull(pfnSectionPoly);
                    }
                }
				if (!bPrivateProp)
				{
					break;
				}
            }
            pModel->endEdit();
            for (auto itr = propVec.cbegin(), itrEnd = propVec.cend(); itr != itrEnd; ++itr)
            {
                updateAxisOffset((*itr)->owner(), dWidth);
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
GTJColumnElevPropInfoWriter::GTJColumnElevPropInfoWriter(std::vector<int> *const pAcceptEntTypes)
    : GMPElevPropInfoWriter(pAcceptEntTypes)
{
    m_AcceptProps.clear();
    m_AcceptProps.push_back(GString::fromStdWString(pfnBottomElev));
}

void GTJColumnElevPropInfoWriter::writeProp(GMPPropPtr pProp, const GString &strNewValue, const GString &strOldValue)
{
    auto const pObject = pProp->owner()->owner();
    if (pObject->type() == IGMPEObject::eoEDO)
    {
        auto const pEdo = dynamic_cast<IGMPElementDrawObj *>(pObject);
        GString strValue = strNewValue.trimmed();
        if (strValue.isEmpty())
        { // 此处是参考了柱子的底标高属性中脚本的逻辑。因为在未决算前，不知道用哪个接口可以获知脚本执行后，属性的值是多少。
            if (pEdo->floor()->code().compare("0") == 0)//基础层code为0
            {
                strValue = QString::fromLocal8Bit("基础底标高");
            }
            else
            {
                strValue = QString::fromLocal8Bit("层底标高");
            }
        }
        double dValue;
        auto const pElevOpr = pEdo->contnr()->model()->oprCenter()->elevOpr();
        if (pElevOpr->checkExpr(pProp->propName(), strValue, pEdo, dValue))
        {
            auto const pShape = pEdo->shape();
            CCoordinates3d oCoord = pShape->coordinate();
            oCoord.Origin.Z = dValue;
            pShape->setCoordinate(oCoord);
            GMPElevPropInfoWriter::writeProp(pProp, strValue, strOldValue);
        }
    }
    else
    {
        GMPElevPropInfoWriter::writeProp(pProp, strNewValue, strOldValue);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
GTJArchBeamElevPropInfoWriter::GTJArchBeamElevPropInfoWriter(IGMPService *const pService,
    std::vector<int> *const pAcceptEntTypes)
    : GMPPropInfoDefaultWriter(pAcceptEntTypes), m_pService(pService)
{
    m_AcceptProps.clear();
    m_AcceptProps.push_back(GString::fromStdWString(pfnStartPtTopElev));
    m_AcceptProps.push_back(GString::fromStdWString(pfnEndPtTopElev));
    m_AcceptProps.push_back(GString::fromStdWString(pfnStartPtBottomElev));
    m_AcceptProps.push_back(GString::fromStdWString(pfnEndPtBottomElev));
    m_pArchBeamCount = -1;
	m_bElevChange = true;
}

bool GTJArchBeamElevPropInfoWriter::beforeWriteProp(vector<GMPPropPtr>& oProps, const GString& strValue, GString& strErrMsg)
{
    /*
    新的多段梁标高修改原则：
        ①每个梁图元斜率已知，参考斜率起点取首段，终点取尾段
        ②第一段梁（首/尾）body无限延伸，剩余斜率一致的梁body是否与延伸后的body重合
        ③以第一段梁起/终点标高，满足条件的最后一段梁终/起点标高的标高差，第一段梁至该段梁长度（Line的长度之和）
    为依据，得到新的斜率Slope。
        ④调整每段满足条件的梁图元至该斜率。
    */
    m_bElevChange = true;
    m_pArchBeamCount = 0;

    m_mapEdoAndTopElevs.clear();
    auto iter = oProps.begin();
    const std::wstring c_sPropName = (*iter)->propName();
    bool bStartTop = (c_sPropName.find(L"StartPt") != std::wstring::npos);

    std::set<IGMPElementDrawObj *> setTermianlEdo;
    IGMPRelationOperator* const pRela = m_pService->model()->oprCenter()->relaOpr();

    auto getNextEdo = [&](IGMPElementDrawObj* pEdo)->IGMPElementDrawObj *
    {
        return bStartTop ? pRela->next(pEdo) : pRela->prev(pEdo);
    };

    auto getElevDis = [&](IGMPElementDrawObj* pEdo)->double
    {
        if (pEdo->elementType() == etFDBeam && pEdo->properties()->asInteger(pfnType) == 2)
        {
            double dElev1 = pEdo->properties()->asDouble(pfnStartPtBottomElev);
            double dElev2 = pEdo->properties()->asDouble(pfnEndPtBottomElev);
            return bStartTop ? (dElev1 - dElev2) : (dElev2 - dElev1);
        }
        IGMPCustomLineSolidShape* pShape = dynamic_cast<IGMPCustomLineSolidShape *>(pEdo->shape());
        return bStartTop ? -pShape->endPtElev() : pShape->endPtElev();
    };

    auto getProp = [&](IGMPElementDrawObj *pEdo, bool bStart)->GMPPropPtr
    {
        if (pEdo->elementType() == etFDBeam && pEdo->properties()->asInteger(pfnType) == 2)
        {
            return bStart ? pEdo->properties()->propByName(pfnStartPtBottomElev) : pEdo->properties()->propByName(pfnEndPtBottomElev);
        }
        return bStart ? pEdo->properties()->propByName(pfnStartPtTopElev) : pEdo->properties()->propByName(pfnEndPtTopElev);
    };

    while (iter != oProps.end())
    {
        IGMPElementDrawObj* c_pEdo = dynamic_cast<IGMPElementDrawObj *>((*iter)->owner()->owner());
        SCOPE_EXIT{++iter;};
        if (!c_pEdo || c_pEdo->element()->refType() == ertComplexBody)
        {
            continue;
        }

        if (!pRela->next(c_pEdo) && !pRela->prev(c_pEdo))
        {
            continue;
        }
        IGMPElementDrawObj* const c_pStartEdo = bStartTop ? pRela->first(c_pEdo) : pRela->last(c_pEdo);
        auto iter_set = setTermianlEdo.find(c_pStartEdo);
        if (iter_set != setTermianlEdo.end())
        {
            continue;
        }
        setTermianlEdo.insert(c_pStartEdo);

        std::map<IGMPElementDrawObj *, pair<double,double>> mapEdos;
        auto const c_pStartShape = dynamic_cast<IGMPCustomLineSolidShape *>(c_pStartEdo->shape());
        const double c_dElevDis = getElevDis(c_pStartEdo);
        double dTotalLength = c_pStartShape->line()->Length();
        const double c_dSlope = c_dElevDis / dTotalLength;
        const double c_dStartElev = getProp(c_pStartEdo, bStartTop)->asDouble();

        mapEdos[c_pStartEdo] = make_pair(dTotalLength, dTotalLength);
        c_pEdo = getNextEdo(c_pStartEdo);
        double dTerminalElev = getProp(c_pStartEdo, !bStartTop)->asDouble();
        IGMPElementDrawObj *pTerminalEdo = nullptr;
        double dSumLength = 0.0;
        while (c_pEdo)
        {
            auto const c_pCurShape = dynamic_cast<IGMPCustomLineSolidShape *>(c_pEdo->shape());
            const double c_dCurElevDis = getElevDis(c_pEdo);
            const double c_dCurLength = c_pCurShape->line()->Length();
            const double c_dCurSlope = c_dCurElevDis / c_dCurLength;
            pTerminalEdo = c_pEdo;
            SCOPE_EXIT
            {
                c_pEdo = getNextEdo(c_pEdo);
            };
            dSumLength += c_dCurLength;
            if (!isEqual(c_dCurSlope, c_dSlope, 0.01))
            {
                continue;
            }

            const double c_dRefElev = c_dStartElev - (dTotalLength + dSumLength) * c_dSlope;
            const double c_dRelElev = getProp(c_pEdo, !bStartTop)->asDouble();
            if (!isEqual(c_dRelElev, c_dRefElev, 1))
            {
                continue;
            }
            dTerminalElev = c_dRelElev;
            dTotalLength += dSumLength;
            dSumLength = 0.0;
            mapEdos[c_pEdo] = make_pair(c_dCurLength, dTotalLength);
        }

        GMPPropPtr pProp = getProp(c_pStartEdo,bStartTop);
        bool bStaus;
        double dNewElev,dOldElev;
        if (!getEdoElev(pProp, strValue, "", bStaus, dNewElev, dOldElev))
        {
            strErrMsg = QString::fromLocal8Bit("标高错误");
            return  false;
        }

        const double dTermianlSlope = (dNewElev - dTerminalElev) / dTotalLength ;

        auto mapEdoIter = mapEdos.begin();
        while (mapEdoIter != mapEdos.end())
        {
            const double c_dElevDis1 = mapEdoIter->second.second * dTermianlSlope;
            const double c_dElevDis2 = mapEdoIter->second.first * dTermianlSlope;

            const double c_dElev1 = dNewElev - c_dElevDis1;
            const double c_dElev2 = c_dElev1 + c_dElevDis2;
            pair<double, double > pairElev = bStartTop ?  make_pair(c_dElev2, c_dElev1) : make_pair(c_dElev1, c_dElev2);

            if (mapEdoIter->first == pTerminalEdo)
            {
                const double c_dNowElev = getProp(pTerminalEdo,!bStartTop)->asDouble();
                const double c_dRefElev = bStartTop ? pairElev.second : pairElev.first;
                if (!isEqual(c_dNowElev, c_dRefElev, 1))
                {
                    ++mapEdoIter;
                    continue;
                }
            }
            m_mapEdoAndTopElevs[mapEdoIter->first] = pairElev;
            ++mapEdoIter;
        }
    }
    if (!m_mapEdoAndTopElevs.empty())
    {
        auto vecPropIter = oProps.begin();
        while (vecPropIter != oProps.end())
        {
            SCOPE_EXIT{++vecPropIter;};
            IGMPElementDrawObj* pEdo = dynamic_cast<IGMPElementDrawObj *>((*vecPropIter)->owner()->owner());
            if (!pEdo)
            {
                continue;
            }

            IGMPCustomLineSolidShape* pShape = dynamic_cast<IGMPCustomLineSolidShape*>(pEdo->shape());
            if (!pShape)
            {
                continue;
            }

            if (pShape->archInfoType() == laiRingBeam)
            {
                std::swap(*vecPropIter, oProps.front());
                break;
            }
        }
    }
    return true;

}

bool GTJArchBeamElevPropInfoWriter::getEdoElev( GMPPropPtr pProp, GString strNewValue, GString strOldValue, bool& bStatus, double& dNewElev, double& dOldElev )
{
    IGMPElementDrawObj * c_pElmDrawObj = dynamic_cast<IGMPElementDrawObj*>(pProp->owner()->owner());
    auto const c_pShape = dynamic_cast<IGMPSectionLineSolidShape *>(c_pElmDrawObj->shape());
    const std::wstring c_pPropName = pProp->propName();
    IGMPElevOperator *const c_pElevOper = m_pService->model()->oprCenter()->elevOpr();

    CVector2d oPoint;
    if (c_pPropName.find(L"StartPt") != std::wstring::npos)
    {
        oPoint = c_pShape->line()->StartPoint();
    }
    else if (c_pPropName.find(L"EndPt") != std::wstring::npos)
    {
        oPoint = c_pShape->line()->EndPoint();
    }
    else
    {
        return false;
    }

    strOldValue.trimmed();
    double dTemp = strOldValue.toDouble(&bStatus) * 1000;
    dOldElev = bStatus ? dTemp : (strOldValue.isEmpty() || strOldValue == "?") ? pProp->asDouble() : c_pElevOper->calcElev(c_pElmDrawObj, oPoint, strOldValue);

    strNewValue.trimmed();
    dTemp = strNewValue.toDouble(&bStatus) * 1000;
    if (!bStatus)
    {
        const auto sTemp = strNewValue.trimmed();
        if (sTemp.isEmpty())
        {
            GString strValue = pProp->schema()->defaultExpr();
            strValue.remove('"');
            c_pElevOper->checkExpr(c_pPropName, strValue, c_pElmDrawObj, dTemp);
        }
        else if (sTemp == "?")
        {
            dTemp = pProp->asDouble();
        }
        else
        {
            dTemp = c_pElevOper->calcElev(c_pElmDrawObj, oPoint, sTemp);
        }
    }
    dNewElev = dTemp;
    return true;
}

void GTJArchBeamElevPropInfoWriter::writeProp(GMPPropPtr pProp, const GString &strNewValue,
    const GString &strOldValue)
{
   bool bBeam = pProp->owner()->owner()->elementType() != etFDBeam;
   IGMPElementDrawObj* const c_pEdo = dynamic_cast<IGMPElementDrawObj *>(pProp->owner()->owner());
   if (!c_pEdo)
   {
       return bBeam ? writeElevProp(pProp, strNewValue, strOldValue) : writeFDBeamElevProp(pProp, strNewValue, strOldValue);
   }
   if (!m_bElevChange)
   {
       return;
   }
   IGMPRelationOperator* const c_pRela = m_pService->model()->oprCenter()->relaOpr();
   auto iter = m_mapEdoAndTopElevs.find(c_pEdo);
   if (iter == m_mapEdoAndTopElevs.end())
   {
       if (!c_pRela->next(c_pEdo) && !c_pRela->prev(c_pEdo))
       {
           return bBeam ? writeElevProp(pProp, strNewValue, strOldValue) : writeFDBeamElevProp(pProp, strNewValue, strOldValue);
       }
       return ;
   }

   bool bStartTop = (pProp->propName().find(L"StartPt") != std::wstring::npos);
   bool bStatus;
   double dNewValue, dOldValue;
   if (!getEdoElev(pProp, strNewValue, strOldValue, bStatus, dNewValue, dOldValue))
   {
       return bBeam ? writeElevProp(pProp, strNewValue, strOldValue) : writeFDBeamElevProp(pProp, strNewValue, strOldValue);
   }
   else
   {
       auto getProp = [&](IGMPElementDrawObj *pEdo, bool bStart)->GMPPropPtr
       {
           if (pEdo->elementType() == etFDBeam && pEdo->properties()->asInteger(pfnType) == 2)
           {
               return bStart ? pEdo->properties()->propByName(pfnStartPtBottomElev) : pEdo->properties()->propByName(pfnEndPtBottomElev);
           }
           return bStart ? pEdo->properties()->propByName(pfnStartPtTopElev) : pEdo->properties()->propByName(pfnEndPtTopElev);
       };

       GString strCurNewValue;
       const double c_dFirstElev = bStartTop ? iter->second.first : iter->second.second;
       strCurNewValue = isEqual(dNewValue, c_dFirstElev, 1) ? strNewValue : QString::number(c_dFirstElev / 1000, 'f', 6);
       bBeam ? writeElevProp(pProp, strCurNewValue, strOldValue) : writeFDBeamElevProp(pProp, strCurNewValue, strOldValue);

       GMPPropPtr pNextProp = getProp(c_pEdo, !bStartTop);//bStartTop ? c_pEdo->properties()->propByName(pfnEndPtTopElev) : c_pEdo->properties()->propByName(pfnStartPtTopElev);
       const double c_dNextElev = !bStartTop ? iter->second.first : iter->second.second;
       const double c_dNextOldElev = pNextProp->asDouble();
       const GString c_strNextOldStr = getProp(c_pEdo, !bStartTop)->asString();//bStartTop ? c_pEdo->properties()->asString(pfnEndPtTopElev) : c_pEdo->properties()->asString(pfnStartPtTopElev);
       if (!isEqual(c_dNextElev, c_dNextOldElev, 1))
       {
           strCurNewValue = QString::number(c_dNextElev / 1000, 'f', 6);
           bBeam ? writeElevProp(pNextProp, strCurNewValue, c_strNextOldStr) : writeFDBeamElevProp(pNextProp, strCurNewValue, c_strNextOldStr);
       }
   }
}

void GTJArchBeamElevPropInfoWriter::writeFDBeamElevProp( GMPPropPtr pProp, const GString &strNewValue, const GString &strOldValue )
{
    const double c_dHeight = pProp->owner()->asInteger(pfnSectionHeight) * 0.001;
    auto const c_strPorpName = pProp->propName();
    byte bType = 0x00;
    // 00/起顶 01/起底 10/终顶 11/终底
    if (c_strPorpName.find(L"StartPt") == std::wstring::npos)
    {
        bType |= 0x10;
    }
    if (c_strPorpName.find(L"Top") == std::wstring::npos)
    {
        bType |= 0x01;
    }

    GMPPropPtr pNextProp;
    if (!(bType & 0x10))
    {
        pNextProp = (!(bType & 0x01)) ? pProp->owner()->propByName(pfnStartPtBottomElev) : pProp->owner()->propByName(pfnStartPtTopElev);
    }
    else
    {
        pNextProp = (!(bType & 0x01)) ? pProp->owner()->propByName(pfnEndPtBottomElev) : pProp->owner()->propByName(pfnEndPtTopElev);
    }
    bool bStatus;
    GString strNextNewValue;
    double dNewElev = strNewValue.toDouble(&bStatus);
    if (bStatus)
    {
        (!(bType & 0x01)) ? dNewElev -= c_dHeight : dNewElev += c_dHeight;
        strNextNewValue = QString::number(dNewElev, 'f', 6);
    }
    else
    {
        const QString c_strHeight = QString::number( c_dHeight, 'f', 6);
        strNextNewValue = (!(bType & 0x01)) ? (strNewValue + "-" + c_strHeight) : (strNewValue + "+" + c_strHeight);
    }
    GString strNextOldValue = pNextProp->asString();

    GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
    GMPPropInfoDefaultWriter::writeProp(pNextProp, strNextNewValue, strNextOldValue);
}
void GTJArchBeamElevPropInfoWriter::writeElevProp( GMPPropPtr pProp, const GString &strNewValue, const GString &strOldValue )
{
    IGMPProperties *const c_pProps = pProp->owner();
    IGMPEObject *const c_pObj = c_pProps->owner();
    if (IGMPEObject::eoEDO == c_pObj->type())
    {
        IGMPElementDrawObj * c_pElmDrawObj = dynamic_cast<IGMPElementDrawObj*>(c_pObj);
        if (c_pElmDrawObj->element()->refType() == ertComplexBody)
        {
            return;
        }
        auto const pOprCenter = c_pElmDrawObj->contnr()->model()->oprCenter();
        auto const pRelaOpr = pOprCenter->relaOpr();
        const std::wstring c_pPropName = pProp->propName();
        auto const c_pShape = dynamic_cast<IGMPSectionLineSolidShape *>(c_pElmDrawObj->shape());
        do
        {
            CVector2d oPoint;
            if (c_pPropName == pfnStartPtTopElev)
            {
                oPoint = c_pShape->line()->StartPoint();
            }
            else if (c_pPropName == pfnEndPtTopElev)
            {
                oPoint = c_pShape->line()->EndPoint();
            }
            else
            {
                break;
            }
            IGMPElevOperator *const c_pElevOper = pOprCenter->elevOpr();
            bool bStatus = false;
            double dTemp = strOldValue.toDouble(&bStatus) * 1000;
            if (!bStatus)
            {
                const auto sTemp = strOldValue.trimmed();
                if (sTemp.isEmpty())
                {
                    dTemp = pProp->asDouble();
                }
                else if (sTemp == "?")
                {
                    dTemp = pProp->asDouble();
                }
                else
                {
                    dTemp = c_pElevOper->calcElev(c_pElmDrawObj, oPoint, strOldValue);
                }
            }
            const double c_dOldValue = dTemp;
            dTemp = strNewValue.toDouble(&bStatus) * 1000;
            if (!bStatus)
            {
                const auto sTemp = strNewValue.trimmed();
                if (sTemp.isEmpty())
                {
                    //                     //为了取得默认值，先将默认值写进数据库，结算；取得默认值；再将之前的值写回去，结算。奇葩
                    //                     GMPElevPropInfoWriter::writeProp(pProp, strNewValue, strOldValue);
                    //                     c_pElmDrawObj->contnr()->model()->calculate();
                    //                     dTemp = c_pElmDrawObj->properties()->propByName(c_pPropName)->asDouble();
                    //                     GMPElevPropInfoWriter::writeProp(pProp, strOldValue, strNewValue);
                    //                     c_pElmDrawObj->contnr()->model()->calculate();
                    GString strValue = pProp->schema()->defaultExpr();
                    strValue.remove('"');
                    c_pElevOper->checkExpr(c_pPropName, strValue, c_pElmDrawObj, dTemp);
                }
                else if (sTemp == "?")
                {
                    dTemp = pProp->asDouble();
                }
                else
                {
                    dTemp = c_pElevOper->calcElev(c_pElmDrawObj, oPoint, sTemp);
                }
            }
            {
                m_pService->model()->beginEdit(true);
                SCOPE_EXIT
                {
                    m_pService->model()->endEdit();
                };
                const double c_dNewValue = dTemp;
                if (ggp::sameValue(c_dOldValue, c_dNewValue, ggp::g_DistEpsilon))
                {
                    break;
                }
                else
                {
                    const int c_nArchInfoType = c_pShape->archInfoType();
                    if (c_nArchInfoType == laiRingBeam)
                    {
                        if (m_pArchBeamCount == 0)
                        {
                            const char *const c_sHint =
                                "修改标高后，如果标高和之前的标高不同，会使拱梁变为平梁或斜梁，是否继续?";
                            if (GMPMessageBox::question(QApplication::activeWindow(),
                                qApp->translate(c_szPropInfoWriter, c_Confrim), QString::fromLocal8Bit(c_sHint),
                                GlodonMessageBox::No | GlodonMessageBox::Yes, GlodonMessageBox::No) == GlodonMessageBox::Yes)
                            {
                                c_pShape->setArchInfoType(laiPlane);
                                m_pService->model()->calculate();
                                const auto sTemp = strNewValue.trimmed();
                                if (sTemp.isEmpty())
                                {
                                    pProp->initDefaultValue();
                                }
                                else
                                {
                                    pProp->setAsString(QString::number(c_dNewValue * 0.001));
                                }
                                m_bElevChange = true;
                                m_pArchBeamCount++;
                                break;
                            }
                            else
                            {
                                m_pArchBeamCount++;
                                m_bElevChange = false;
                                return;
                            }
                        }
                        else
                        {
                            if (m_bElevChange)
                            {
                                c_pShape->setArchInfoType(laiPlane);
                                const auto sTemp = strNewValue.trimmed();
                                if (sTemp.isEmpty())
                                {
                                    pProp->initDefaultValue();
                                }
                                else
                                {
                                    pProp->setAsString(QString::number(c_dNewValue * 0.001));
                                }
                                break;
                            }
                            else
                            {
                                return;
                            }
                        }
                    }
                    else if (c_nArchInfoType == laiBeam)
                    {
                        if (c_pPropName == pfnStartPtTopElev)
                        {
                            if (bStatus)
                            {
                                pProp->setAsString(QString::number(c_dNewValue * 0.001));
                            }
                            else
                            {
                                if (strNewValue.isEmpty())
                                {
                                    pProp->initDefaultValue();
                                }
                                else
                                {
                                    pProp->setAsString(strNewValue);
                                }
                            }
                            c_pElmDrawObj->contnr()->model()->calculate();
                            updateSemiArchConfig(c_pElmDrawObj, c_pPropName);
                            return;
                        }
                    }
                }
                writeLaiPlaneProp(pProp, strNewValue, strOldValue);
                c_pElmDrawObj->contnr()->model()->calculate();
                updateSemiArchConfig(c_pElmDrawObj, c_pPropName);
            }
            refreshBeamHanger(m_pService, c_pObj);
            return;
        } while (false);
    }
    writeLaiPlaneProp(pProp, strNewValue, strOldValue);
}

void GTJArchBeamElevPropInfoWriter::refreshBeamHanger(IGMPService *const pService,
    IGMPEObject *const pObject) const
{
    if (pObject->type() == IGMPEObject::eoEDO)
    {
        auto const pEdo = dynamic_cast<IGMPElementDrawObj *>(pObject);
        if (pEdo != nullptr && pEdo->elementType() == etBeam)
        {
            auto const pPropVec = pEdo->properties();
            if (pPropVec->hasProp(GTJDBCConsts::pfnIdentified)
                && pPropVec->asBoolean(GTJDBCConsts::pfnIdentified)
                && pPropVec->hasProp(GTJDBCConsts::pfnHangPosition))
            {
                const QString strValue = pPropVec->asString(GTJDBCConsts::pfnHangPosition);
                if (!strValue.isEmpty())
                {
                    pService->model()->oprCenter()->displayOpr()->refreshEdoDisplay(pEdo);
                }
            }
        }
    }
}

bool GTJArchBeamElevPropInfoWriter::validateProp(GMPPropPtr pProp, const GString &strNewValue, GString &strErrMsg)
{
    if (GMPPropInfoDefaultWriter::validateProp(pProp, strNewValue, strErrMsg))
    {
        // strValue 诸如7/1/0是不合法的，需要解析判断
        GString strValue(strNewValue.trimmed());
        if (!strValue.isEmpty())
        {
            const QStringList valueList = strValue.split(QChar('/'), QString::SkipEmptyParts);
            bool bStatus = false;
            double dValue;
            auto itr = valueList.cbegin();
            auto const itrEnd = valueList.cend();
            for (++itr; itr != itrEnd; ++itr)
            {
                try
                {
                    dValue = std::stod(itr->toStdWString(), nullptr);
                    if (ggp::isZero(dValue, 0.001))
                    {
                        bStatus = true;
                        break;
                    }
                }
                catch (const std::invalid_argument &)
                {
                    bStatus = true;
                    break;
                }
                catch (const std::out_of_range &)
                {
                    bStatus = true;
                    break;
                }
            }
            if (bStatus)
            {
                strErrMsg = QString::fromLocal8Bit("标高输入错误");
                return false;
            }
        }
        return true;
    }
    return false;
}

bool GTJArchBeamElevPropInfoWriter::afterWriteProp(vector<GMPPropPtr>& oProps,
    const GString& strValue, GString& strErrMsg)
{
    refreshBeamHanger(m_pService, oProps.front()->owner()->owner());
    return true;
}

void GTJArchBeamElevPropInfoWriter::updateElevOfCommonArchBeam(IGMPElementDrawObj *const pElmDrawObj)
{
    IGMPSectionLineSolidShape *const c_pShape = dynamic_cast<IGMPSectionLineSolidShape *>(pElmDrawObj->shape());
    const CCoordinates3d c_oCoord = c_pShape->coordinate();
    const double c_dEndElev = c_pShape->endPtElev();
    CLine2d *const c_pLine2D = dynamic_cast<CLine2d *>(c_pShape->line().get());
    const double c_dBeamHeight = pElmDrawObj->properties()->asDouble(pfnStartPtTopElev);

    double dTemp = c_pLine2D->Length();
    double c_dLength = std::sqrt(dTemp * dTemp + c_dEndElev * c_dEndElev);
    const double c_dSinTheta = c_dEndElev / c_dLength;
    const double c_dCosTheta = std::sqrt(1 - c_dSinTheta * c_dSinTheta);
    c_dLength = dTemp;

    IGMPRingBeamLineArchInfo *const c_pArchInfo = dynamic_cast<IGMPRingBeamLineArchInfo *>(c_pShape->getArchInfo());
    const double c_dArchHeight = c_pArchInfo->archHeight();
    dTemp = std::fabs(c_pArchInfo->radius());
    const double c_dArchRadius = ((c_dArchHeight > 0) ? dTemp : -dTemp);
    const double c_dHalfChord = c_pArchInfo->halfChordLength();
    const CVector2d c_oBasePt = c_pArchInfo->basePoint();
    const double c_dCenterX = c_pLine2D->GetNearestT(c_oBasePt) - c_pLine2D->StartT();

    dTemp = getCommonArchBeamPtHeight(c_dLength, c_dBeamHeight, c_dCenterX - c_dHalfChord,
        c_dCenterX + c_dHalfChord, c_dArchRadius, 0);
    dTemp = dTemp * c_dCosTheta + c_oCoord.Origin.Z;
    double dElev = pElmDrawObj->properties()->asDouble(pfnStartPtTopElev);
    if (!ggp::sameValue(dElev, dTemp, ggp::g_DistEpsilon))
    {
        pElmDrawObj->properties()->propByName(pfnStartPtTopElev)->setAsString(QString::number(dTemp * 0.001));
    }

    dTemp = getCommonArchBeamPtHeight(c_dLength, c_dBeamHeight, c_dCenterX - c_dHalfChord,
        c_dCenterX + c_dHalfChord, c_dArchRadius, c_dLength);
    dTemp = dTemp * c_dCosTheta + c_dLength * c_dSinTheta + c_oCoord.Origin.Z;
    dElev = pElmDrawObj->properties()->asDouble(pfnEndPtTopElev);
    if (!ggp::sameValue(dElev, dTemp, ggp::g_DistEpsilon))
    {
        pElmDrawObj->properties()->propByName(pfnEndPtTopElev)->setAsString(QString::number(dTemp * 0.001));
    }
}

void GTJArchBeamElevPropInfoWriter::updateSemiArchConfig(IGMPElementDrawObj *const pElmDrawObj,
    const std::wstring &pPropName)
{
    if ((pPropName == pfnStartPtTopElev) || (pPropName == pfnEndPtTopElev))
    {
        IGMPSectionLineSolidShape *const c_pShape = dynamic_cast<IGMPSectionLineSolidShape *>(pElmDrawObj->shape());
        if (c_pShape->archInfoType() == laiBeam)
        {
            const double c_dLength = c_pShape->line()->Length();
            IGMPBeamLineArchInfo *const c_pArchInfo = dynamic_cast<IGMPBeamLineArchInfo *>(c_pShape->getArchInfo());
            double dTemp = c_pArchInfo->endDist(true);
            if (dTemp - c_dLength > 0.5)//半拱
            {
                const double c_dActualLength = c_dLength;
                const double c_dTopStart = c_pArchInfo->startDist(true);
                const double c_dTopEnd = c_dActualLength * 2 - c_dTopStart;

                double dArchRadius = c_pArchInfo->radius(true);
                double dArchHeight = c_pArchInfo->endDist(false);
                const bool c_bDefinedByRadius = (c_pArchInfo->startDist(false) > 1);
                if (!c_bDefinedByRadius)
                {
                    dTemp = c_dActualLength - c_dTopStart;
                    dArchRadius = (dTemp * dTemp / dArchHeight + dArchHeight) * 0.5;
                }
                c_pArchInfo->defArchByRadius(true, c_dTopStart, c_dTopEnd, dArchRadius);
            }
        }
    }
}
// 
// bool GTJArchBeamElevPropInfoWriter::isLinkageBeam(IGMPElementDrawObj *pEdo)
// {
//     IGMPRelationOperator *const pRelaOpr = pEdo->contnr()->model()->oprCenter()->relaOpr();
//     IGMPElementDrawObj *pPrev = pRelaOpr->first(pEdo);
//     if (pPrev->shape()->shapeClass() == scLine)
//     { //处理中间梁段标高，非连接梁不需要单独计算标高
//         return (pRelaOpr->next(pPrev) != nullptr);
//     }
//     return false;
// }
// 
// bool GTJArchBeamElevPropInfoWriter::isParallelBeam(IGMPElementDrawObj *pEdo)
// {
//     IGMPRelationOperator *const pRelaOpr = pEdo->contnr()->model()->oprCenter()->relaOpr();
//     IGMPElementDrawObj* pPrev = pRelaOpr->first(pEdo);
//     IGMPCustomLineSolidShape *pShapePrev = dynamic_cast<IGMPCustomLineSolidShape *>(pPrev->shape());
//     if (pShapePrev == nullptr)
//         return false;
// 
//     //处理中间梁段标高
//     IGMPElementDrawObj *pEdoCurr = pPrev;
//     while (pEdoCurr != nullptr)
//     {
//         IGMPCustomLineSolidShape *pShapeNext = dynamic_cast<IGMPCustomLineSolidShape*>(pEdoCurr->shape());
//         if (pShapeNext != nullptr)
//         {
//             //判断是否平行
//             if (!GMPPositionFunc2d::isParallel(pShapePrev->line().get(), pShapeNext->line().get(), ggp::g_DistEpsilon))
//             {
//                 return false;
//             }
//         }
//         pEdoCurr = pRelaOpr->next(pEdoCurr);
//     }
//     return true;
// }
// 
// bool GTJArchBeamElevPropInfoWriter::isAllLine2dBeam(IGMPElementDrawObj * pEdo)
// {
//     IGMPRelationOperator *const pRelaOpr = pEdo->contnr()->model()->oprCenter()->relaOpr();
//     IGMPElementDrawObj* pPrev = pRelaOpr->first(pEdo);
//     IGMPCustomLineSolidShape *pShapePrev = dynamic_cast<IGMPCustomLineSolidShape*>(pPrev->shape());
//     if (pShapePrev == nullptr)
//         return false;
// 
//     IGMPElementDrawObj *pNext = pPrev;
//     while (pNext != nullptr)
//     {
//         IGMPCustomLineSolidShape *pShapeNext = dynamic_cast<IGMPCustomLineSolidShape*>(pNext->shape());
//         if (pShapeNext == nullptr)
//             continue;
// 
//         //判断标高是否相等
//         //if (pShapeNext->line()->Type() != Line2dType)
//         //{
//         //    return false;
//         //}
//         pNext = pRelaOpr->next(pNext);
//     }
//     return true;
// }
// 
// bool GTJArchBeamElevPropInfoWriter::isPlaneBeam(IGMPElementDrawObj * pEdo)
// {
//     IGMPRelationOperator *const pRelaOpr = pEdo->contnr()->model()->oprCenter()->relaOpr();
//     IGMPElementDrawObj* pPrev = pRelaOpr->first(pEdo);
//     IGMPCustomLineSolidShape *pShapePrev = dynamic_cast<IGMPCustomLineSolidShape*>(pPrev->shape());
//     if (pShapePrev == nullptr)
//         return false;
// 
//     //处理中间梁段标高
//     IGMPElementDrawObj *pNext = pPrev;
//     while (pNext != nullptr)
//     {
//         IGMPCustomLineSolidShape *pShapeNext = dynamic_cast<IGMPCustomLineSolidShape*>(pNext->shape());
//         if (pShapeNext == nullptr)
//             continue;
// 
//         //判断标高是否相等
//         if (!pShapePrev->coordinate().Z.IsEqual(pShapeNext->coordinate().Z, g_DistEpsilon))
//         {
//             return false;
//         }
//         pNext = pRelaOpr->next(pNext);
//     }
//     return true;
// }
// 
// void GTJArchBeamElevPropInfoWriter::calcBeamElve(GMPPropPtr pProp, IGMPElementDrawObj * pEdo,
//     const GString& strNewValue, const GString& strOldValue)
// {
//     //处理清空后恢复默认值
//     if (strNewValue.trimmed().isEmpty())
//     {
//         if (pProp->propName() == pfnStartPtTopElev)
//         {
//             pProp->owner()->propByName(pfnStartPtTopElev)->initDefaultValue();
//         }
//         else
//         {
//             pProp->owner()->propByName(pfnEndPtTopElev)->initDefaultValue();
//         }
//         return;
//     }
//     IGMPRelationOperator *const pRelaOpr = pEdo->contnr()->model()->oprCenter()->relaOpr();
//     IGMPElementDrawObj *const pPrev = pRelaOpr->first(pEdo);
//     IGMPElementDrawObj *const pLast = pRelaOpr->last(pEdo);
//     if (pPrev == nullptr || pLast == nullptr)
//     {
//         return;
//     }
// 
//     IGMPCustomLineSolidShape *const pShapePrev = dynamic_cast<IGMPCustomLineSolidShape*>(pPrev->shape());
//     IGMPCustomLineSolidShape *const pShapeLast = dynamic_cast<IGMPCustomLineSolidShape*>(pLast->shape());
//     if (pShapeLast == nullptr || pShapePrev == nullptr)
//     {
//         return;
//     }
// 
//     CCurve2dPtr pPrevLine = pShapePrev->line();
//     CCurve2dPtr pLastLine = pShapeLast->line();
// 
//     pPrevLine = ggp::OrientProjectCurve2d(pShapePrev->coordinate(), pPrevLine.get(), CCoordinates3d(), CVector3d::UnitZ);
//     pLastLine = ggp::OrientProjectCurve2d(pShapeLast->coordinate(), pLastLine.get(), CCoordinates3d(), CVector3d::UnitZ);
// 	if (nullptr == pPrevLine || nullptr == pLastLine)
// 	{
// 		return;
// 	}
// 
//     const CVector2d c_StartPrev = pPrevLine->StartPoint();
//     const CVector2d c_EndLast = pLastLine->EndPoint();
// 
//     double dLength = std::numeric_limits<double>::epsilon();
//     IGMPCustomLineSolidShape *pShapeNext;
//     IGMPElementDrawObj* pNext = pPrev;
//     do {
//         pShapeNext = dynamic_cast<IGMPCustomLineSolidShape *>(pNext->shape());
//         if (pShapeNext != nullptr)
//         {
//             dLength += pShapeNext->line()->Length();
//             pNext = pRelaOpr->next(pNext);
//         }
//     } while (pNext != nullptr);
//     double dLengthNext = pShapePrev->line()->Length();
//     //处理起点标高变化
//     IGMPElevOperator *const c_pElevOper = pPrev->contnr()->model()->oprCenter()->elevOpr();
//     double const c_dEndOldValue = pLast->properties()->asInteger(pfnEndPtTopElev);
//     double const c_dStartOldValue = pPrev->properties()->asInteger(pfnStartPtTopElev);
//     double c_dStartNewValue = c_pElevOper->calcElev(pPrev, c_StartPrev, strNewValue);//c_pElevOper->calcElev(pPrev, c_StartPrev, strOldValue);
//     double c_dEndNewValue = c_pElevOper->calcElev(pPrev, c_EndLast, strNewValue);
//     //
//     if (pProp->propName() == pfnStartPtTopElev)
//     {
//         //设置起点标高
//         GMPPropInfoDefaultWriter::writeProp(pPrev->properties()->propByName(pProp->propName()), strNewValue, strOldValue);
//         calcNormalBeamElve(pProp, pPrev, dLength, dLengthNext, c_dEndOldValue, c_dStartNewValue);
//     }
//     else
//     {
//         //设置终点标高
//         GMPPropInfoDefaultWriter::writeProp(pLast->properties()->propByName(pProp->propName()), strNewValue, strOldValue);
//         calcSpecialBeamElve(pProp, pPrev, dLength, dLengthNext, c_dStartOldValue, c_dEndNewValue);
//     }
// }
// 
// void GTJArchBeamElevPropInfoWriter::calcNormalBeamElve(GMPPropPtr pProp, IGMPElementDrawObj *pEdo,
//     double dLength, double dLengthNext, double dOldElve, double dNewElve)
// {
//     auto safeSetProp = [&] (GMPPropPtr prop, const GString&sValue) {
//         // 单位化
//         GString strTransfer;
//         if (!m_pUnitTransfer || !m_pUnitTransfer->display2Store(prop, sValue, strTransfer, false))
//         {
//             strTransfer = sValue.trimmed();
//         }
//         // 根据数据类型设置值，标高目前只会是gmdtText，因为数据库中对应的是Memo
//         prop->setAsString(strTransfer);
//     };
//     //设置起点标高
//     double dTempElve = 0;
//     double dTemp = dNewElve - dOldElve;
//     if (ggp::IsGreaterEqualThan(dNewElve, 0, g_DistEpsilon))
//     {
//         if (dNewElve < dOldElve)
//         {
//             if (!ggp::isZero(dLength, ggp::g_DoubleResolution))
//             {
//                 //lint -e795
//                 dTempElve = (dLengthNext) * fabs(dTemp) / dLength;
//                 //lint +e795
//             }
// 
//             if (dTempElve > dOldElve)
//             {
//                 dTempElve = dOldElve - dTempElve;
//             }
//             else
//             {
//                 dTempElve = dTempElve + dNewElve;
//             }
//         }
//         else
//         {
//             if (!ggp::isZero(dLength, ggp::g_DoubleResolution))
//             {
//                 //lint -e795
//                 dTempElve = (dLength - dLengthNext)* fabs(dTemp) / dLength;
//                 //lint +e795
//             }
//             dTempElve += dOldElve;
//         }
//     }
//     else
//     {
//         if (dNewElve < dOldElve)
//         {
//             if (!ggp::isZero(dLength, ggp::g_DoubleResolution))
//             {
//                 //lint -e795
//                 dTempElve = (dLength - dLengthNext) * fabs(dTemp) / dLength;
//                 //lint +e795
//             }
// 
//             if (ggp::IsGreaterEqualThan(dOldElve, 0, g_DistEpsilon))
//             {
//                 dTempElve = dOldElve - dTempElve;
//             }
//             else
//             {
//                 dTempElve = dOldElve - dTempElve;
//             }
//         }
//         else
//         {
//             if (!ggp::isZero(dLength, ggp::g_DoubleResolution))
//             {
//                 //lint -e795
//                 dTempElve = (dLengthNext) * fabs(dTemp) / dLength;
//                 //lint +e795
//             }
//             dTempElve = -dTempElve + dNewElve;
//         }
//     }
//     //设置起点标高
//     safeSetProp(pEdo->properties()->propByName(pfnEndPtTopElev), QString::number(dTempElve / 1000));
//     //循环计算这些点标高
//     IGMPRelationOperator *pRelaOpr = pEdo->contnr()->model()->oprCenter()->relaOpr();
//     IGMPElementDrawObj *pNext = pRelaOpr->next(pEdo);
//     while (pNext != nullptr)
//     {
//         //起点坐标
//         safeSetProp(pNext->properties()->propByName(pProp->propName()), QString::number(dTempElve / 1000));
// 
//         //计算下个坐标点
//         IGMPCustomLineSolidShape *pShapeNext = dynamic_cast<IGMPCustomLineSolidShape*>(pNext->shape());
//         if (pShapeNext == nullptr)
//         {
//             pNext = pRelaOpr->next(pNext);
//             continue;
//         }
//         //根据不同坐标计算不同标高
//         dLengthNext += pShapeNext->line()->ApprLength();
//         if (ggp::IsGreaterEqualThan(dNewElve, 0, g_DistEpsilon))
//         {
//             if (dNewElve < dOldElve)
//             {
//                 if (!ggp::isZero(dLength, ggp::g_DoubleResolution))
//                 {
//                     //lint -e795
//                     dTempElve = (dLengthNext) * fabs(dTemp) / dLength;
//                     //lint +e795
//                 }
// 
//                 if (dTempElve > dOldElve)
//                 {
//                     dTempElve = dOldElve - dTempElve;
//                 }
//                 else
//                 {
//                     dTempElve = dTempElve + dNewElve;
//                 }
//                 if (ggp::isZero(dTempElve, g_DistEpsilon) ||
//                     ggp::IsGreaterEqualThan(dTempElve, dOldElve, g_DistEpsilon))
//                     break;
//             }
//             else
//             {
//                 if (!ggp::isZero(dLength, ggp::g_DoubleResolution))
//                 {
//                     //lint -e795
//                     dTempElve = (dLength - dLengthNext)* fabs(dTemp) / dLength;
//                     //lint +e795
//                 }
//                 dTempElve += dOldElve;
// 
//                 if (ggp::IsLessEqualThan(dTempElve, dOldElve, g_DistEpsilon) ||
//                     ggp::IsGreaterEqualThan(dTempElve, dNewElve, g_DistEpsilon))
//                     break;
//             }
//         }
//         else
//         {
//             if (dNewElve < dOldElve)
//             {
//                 if (!ggp::isZero(dLength, ggp::g_DoubleResolution))
//                 {
//                     //lint -e795
//                     dTempElve = (dLength - dLengthNext) * fabs(dTemp) / dLength;
//                     //lint +e795
//                 }
// 
//                 if (ggp::IsGreaterEqualThan(dOldElve, 0, g_DistEpsilon))
//                 {
//                     dTempElve = dOldElve - dTempElve;
// 
//                 }
//                 else
//                 {
//                     dTempElve = dOldElve - dTempElve;
//                 }
//                 if (ggp::isZero(dTempElve, g_DistEpsilon) || ggp::IsGreaterEqualThan(dTempElve, dOldElve, g_DistEpsilon))
//                     break;
//             }
//             else
//             {
//                 if (!ggp::isZero(dLength, ggp::g_DoubleResolution))
//                 {
//                     //lint -e795
//                     dTempElve = (dLengthNext) * fabs(dTemp) / dLength;
//                     //lint +e795
//                 }
//                 dTempElve = -dTempElve + dNewElve;
// 
//                 if (ggp::IsLessEqualThan(dTempElve, dOldElve, g_DistEpsilon) || ggp::IsGreaterEqualThan(dTempElve, dNewElve, g_DistEpsilon))
//                     break;
//             }
//         }
//         //终点坐标
//         safeSetProp(pNext->properties()->propByName(pfnEndPtTopElev), QString::number(dTempElve / 1000));
//         pNext = pRelaOpr->next(pNext);
//     }
// }
// 
// void GTJArchBeamElevPropInfoWriter::calcSpecialBeamElve(GMPPropPtr pProp, IGMPElementDrawObj *pEdo, double dLength,
//     double dLengthNext, double dOldElve, double dNewElve)
// {
//     auto safeSetProp = [&] (GMPPropPtr prop, const GString&sValue) {
//         // 单位化
//         GString strTransfer;
//         if (!m_pUnitTransfer || !m_pUnitTransfer->display2Store(prop, sValue, strTransfer, false))
//         {
//             strTransfer = sValue.trimmed();
//         }
//         // 根据数据类型设置值，标高目前只会是gmdtText，因为数据库中对应的是Memo
//         prop->setAsString(strTransfer);
//         //prop->setAsDouble(strTransfer.toDouble() * 1000);
//     };
// 
//     //设置起点标高
//     double dTempElve = 0;
//     double dTemp = dNewElve - dOldElve;
//     if (ggp::IsGreaterEqualThan(dNewElve, 0, g_DistEpsilon))
//     {
//         if (dNewElve > dOldElve)
//         {
//             if (!ggp::isZero(dLength, ggp::g_DoubleResolution))
//             {
//                 //lint -e795
//                 dTempElve = (dLengthNext) * fabs(dTemp) / dLength;
//                 //lint +e795
//             }
//             dTempElve = dOldElve + dTempElve;
//         }
//         else
//         {
//             if (!ggp::isZero(dLength, ggp::g_DoubleResolution))
//             {
//                 //lint -e795
//                 dTempElve = (dLength - dLengthNext)* fabs(dTemp) / dLength;
//                 //lint +e795
//             }
//             dTempElve += dNewElve;
//         }
//     }
//     else
//     {
//         if (dNewElve > dOldElve)
//         {
//             if (!ggp::isZero(dLength, ggp::g_DoubleResolution))
//             {
//                 //lint -e795
//                 dTempElve = (dLength - dLengthNext) * fabs(dTemp) / dLength;
//                 //lint +e795
//             }
//             dTempElve = dNewElve - dTempElve;
//         }
//         else
//         {
//             if (!ggp::isZero(dLength, ggp::g_DoubleResolution))
//             {
//                 //lint -e795
//                 dTempElve = (dLengthNext) * fabs(dTemp) / dLength;
//                 //lint +e795
//             }
// 
//             if (ggp::IsGreaterEqualThan(dOldElve, 0, g_DistEpsilon))
//             {
//                 dTempElve = dOldElve - dTempElve;
//             }
//             else
//             {
//                 dTempElve = -dTempElve + dOldElve;
//             }
//         }
//     }
//     //设置起点标高
//     safeSetProp(pEdo->properties()->propByName(pfnEndPtTopElev), QString::number(dTempElve / 1000));
//     //循环计算这些点标高
//     IGMPRelationOperator *pRelaOpr = pEdo->contnr()->model()->oprCenter()->relaOpr();
//     IGMPElementDrawObj *pNext = pRelaOpr->next(pEdo);
//     while (pNext != nullptr)
//     {
//         //起点坐标
//         safeSetProp(pNext->properties()->propByName(pfnStartPtTopElev), QString::number(dTempElve / 1000));
// 
//         //计算下个坐标点
//         IGMPCustomLineSolidShape *pShapeNext = dynamic_cast<IGMPCustomLineSolidShape*>(pNext->shape());
//         if (pShapeNext == nullptr)
//         {
//             pNext = pRelaOpr->next(pNext);
//             continue;
//         }
//         //根据不同坐标计算不同标高
//         dLengthNext += pShapeNext->line()->ApprLength();
//         if (ggp::IsGreaterEqualThan(dNewElve, 0, g_DistEpsilon))
//         {
//             if (dNewElve > dOldElve)
//             {
//                 if (!ggp::isZero(dLength, ggp::g_DoubleResolution))
//                 {
//                     //lint -e795
//                     dTempElve = (dLengthNext) * fabs(dTemp) / dLength;
//                     //lint +e795
//                 }
//                 dTempElve = dOldElve + dTempElve;
// 
//                 if (ggp::IsLessEqualThan(dTempElve, dOldElve, g_DistEpsilon) ||
//                     ggp::IsGreaterEqualThan(dTempElve, dNewElve, g_DistEpsilon))
//                     break;
//             }
//             else
//             {
//                 if (!ggp::isZero(dLength, ggp::g_DoubleResolution))
//                 {
//                     //lint -e795
//                     dTempElve = (dLength - dLengthNext)* fabs(dTemp) / dLength;
//                     //lint +e795
//                 }
//                 dTempElve += dNewElve;
// 
//                 if (ggp::isZero(dTempElve, g_DistEpsilon) ||
//                     ggp::IsGreaterEqualThan(dTempElve, dOldElve, g_DistEpsilon))
//                     break;
//             }
//         }
//         else
//         {
//             if (dNewElve > dOldElve)
//             {
//                 if (!ggp::isZero(dLength, ggp::g_DoubleResolution))
//                 {
//                     //lint -e795
//                     dTempElve = (dLength - dLengthNext) * fabs(dTemp) / dLength;
//                     //lint +e795
//                 }
// 
//                 dTempElve = dNewElve - dTempElve;
//                 if (ggp::isZero(dTempElve, g_DistEpsilon) ||
//                     ggp::IsLessEqualThan(dTempElve, 0, g_DistEpsilon))
//                     break;
//             }
//             else
//             {
//                 if (!ggp::isZero(dLength, ggp::g_DoubleResolution))
//                 {
//                     //lint -e795
//                     dTempElve = (dLengthNext) * fabs(dTemp) / dLength;
//                     //lint +e795
//                 }
// 
//                 if (ggp::IsGreaterEqualThan(dOldElve, 0, g_DistEpsilon))
//                 {
//                     dTempElve = dOldElve - dTempElve;
//                 }
//                 else
//                 {
//                     dTempElve = -dTempElve + dOldElve;
//                 }
//                 if (ggp::IsLessEqualThan(dTempElve, dNewElve, g_DistEpsilon) ||
//                     ggp::IsGreaterEqualThan(dTempElve, dOldElve, g_DistEpsilon))
//                     break;
//             }
//         }
//         //终点坐标
//         //如果是最后一跨就不修改了，应为之前都已经修改过了
//         if (pRelaOpr->next(pNext))
//         {
//             safeSetProp(pNext->properties()->propByName(pProp->propName()), QString::number(dTempElve / 1000));
//         }
//         pNext = pRelaOpr->next(pNext);
//     }
// }
// 
// void GTJArchBeamElevPropInfoWriter::calcFDBeamElve(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
// {
//     Q_ASSERT(pProp);
//     IGMPProperties *const pPropList = pProp->owner();
//     Q_ASSERT(pPropList);
// 
//     auto safeSetProp = [&] (GMPPropPtr prop, const GString&sValue) {
//         // 单位化
//         GString strTransfer;
//         if (!m_pUnitTransfer || !m_pUnitTransfer->display2Store(prop, sValue, strTransfer, false))
//         {
//             strTransfer = sValue.trimmed();
//         }
//         // 根据数据类型设置值，标高目前只会是gmdtText，因为数据库中对应的是Memo
//         prop->setAsString(strTransfer);
//     };
// 
//     //先做默认值和空值的处理
//     // 恢复默认值
//     //这里做了假设认为高度的单位是mm而标高的单位是m
//     const std::wstring &sName = pProp->propName();
//     if (strNewValue.trimmed().isEmpty())
//     {
//         if (sName == pfnStartPtTopElev)
//         {
//             pProp->owner()->propByName(pfnStartPtBottomElev)->initDefaultValue();
//         }
//         else if (sName == pfnStartPtBottomElev)
//         {
//             pProp->owner()->propByName(pfnStartPtTopElev)->initDefaultValue();
//         }
//         else if (sName == pfnEndPtTopElev)
//         {
//             pProp->owner()->propByName(pfnEndPtBottomElev)->initDefaultValue();
//         }
//         else if (sName == pfnEndPtBottomElev)
//         {
//             pProp->owner()->propByName(pfnEndPtTopElev)->initDefaultValue();
//         }
//         pProp->initDefaultValue();
//         return;
//     }
// 
//     double dSectionHeight = pPropList->asInteger(pfnSectionHeight) * 0.001;
// 
//     //2代表承台梁，只显示底标高
//     if (pPropList->hasProp(pfnType) && pPropList->asInteger(pfnType) == 2)
//     {
//         if (sName == pfnStartPtBottomElev)
//         {
//             safeSetProp(pPropList->propByName(pfnStartPtTopElev), strNewValue + "+" + QString::number(dSectionHeight));
//         }
//         else if (pfnEndPtBottomElev == sName)
//         {
//             safeSetProp(pPropList->propByName(pfnEndPtTopElev), strNewValue + "+" + QString::number(dSectionHeight));
//         }
//     }
//     else
//     {
//         if (sName == pfnStartPtTopElev)
//         {
//             safeSetProp(pPropList->propByName(pfnStartPtBottomElev), strNewValue + "-" + QString::number(dSectionHeight));
//         }
//         else if (pfnEndPtTopElev == sName)
//         {
//             safeSetProp(pPropList->propByName(pfnEndPtBottomElev), strNewValue + "-" + QString::number(dSectionHeight));
//         }
//     }
// 
//     safeSetProp(pProp, strNewValue);
// }

void GTJArchBeamElevPropInfoWriter::writeLaiPlaneProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    Q_ASSERT(pProp);
    GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
}
// 
// void GTJArchBeamElevPropInfoWriter::writeLinkageProp(IGMPElementDrawObj *pEdo, GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
// {
//     if (isLinkageBeam(pEdo))
//     {
//         if (isPlaneBeam(pEdo))
//         {
//             if (isParallelBeam(pEdo) && isAllLine2dBeam(pEdo))
//             {
//                 //二维，三维平行的直线梁
//                 if (pEdo->elementType() == etFDBeam)
//                 {
//                     calcLinkFDBeamElve(pProp, pEdo, strNewValue, strOldValue);
//                 }
//                 else
//                 {
//                     calcBeamElve(pProp, pEdo, strNewValue, strOldValue);
//                 }
//             }
//             else
//             {
//                 //三维平行的非曲线的折线梁
//                 if (pEdo->elementType() == etFDBeam)
//                 {
//                     calcFDBeamElve(pProp, strNewValue, strOldValue);
//                 }
//                 else
//                 {
//                     GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
//                 }
// 
//             }
//         }
//         else
//         {
//             writeCommonProp(pEdo, pProp, strNewValue, strOldValue);
//         }
//     }
//     else
//     {
//         auto const nElmType = pEdo->elementType();
//         if (nElmType == etFDBeam)
//         {
//             calcFDBeamElve(pProp, strNewValue, strOldValue);
//         }
//         else
//         {
//             GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
//         }
//     }
// }
// 
// void GTJArchBeamElevPropInfoWriter::calcLinkFDBeamElve(GMPPropPtr pProp, IGMPElementDrawObj * pEdo, const GString& strNewValue, const GString& strOldValue)
// {
//     Q_ASSERT(pProp);
//     Q_ASSERT(pProp->owner());
// 
//     auto safeSetProp = [&] (GMPPropPtr prop, const GString&sValue) {
//         // 单位化
//         GString strTransfer;
//         if (!m_pUnitTransfer || !m_pUnitTransfer->display2Store(prop, sValue, strTransfer, false))
//         {
//             strTransfer = sValue.trimmed();
//         }
//         // 根据数据类型设置值，标高目前只会是gmdtText，因为数据库中对应的是Memo
//         prop->setAsString(strTransfer);
//     };
//     //先做默认值和空值的处理
//     // 恢复默认值
//     //这里做了假设认为高度的单位是mm而标高的单位是m
//     const std::wstring &sName = pProp->propName();
//     if (strNewValue.trimmed().isEmpty())
//     {
//         if (sName == pfnStartPtTopElev)
//         {
//             pProp->owner()->propByName(pfnStartPtBottomElev)->initDefaultValue();
//         }
//         else if (sName == pfnStartPtBottomElev)
//         {
//             pProp->owner()->propByName(pfnStartPtTopElev)->initDefaultValue();
//         }
//         else if (sName == pfnEndPtTopElev)
//         {
//             pProp->owner()->propByName(pfnEndPtBottomElev)->initDefaultValue();
//         }
//         else if (sName == pfnEndPtBottomElev)
//         {
//             pProp->owner()->propByName(pfnEndPtTopElev)->initDefaultValue();
//         }
//         pProp->initDefaultValue();
//         return;
//     }
// 
//     IGMPRelationOperator *pRelaOpr = pEdo->contnr()->model()->oprCenter()->relaOpr();
//     IGMPElementDrawObj* pPrev = pRelaOpr->first(pEdo);
//     IGMPElementDrawObj* pLast = pRelaOpr->last(pEdo);
// 
//     double dStartSectionHeight = pPrev->properties()->asInteger(pfnSectionHeight) / 1000.0;
//     double dEndSectionHeight = pLast->properties()->asInteger(pfnSectionHeight) / 1000.0;
//     if (pPrev == nullptr || pLast == nullptr)
//         return;
// 
//     if (pPrev->iD() != pEdo->iD())
//     {
//         return;
//     }
// 
//     IGMPCustomLineSolidShape *pShapePrev = dynamic_cast<IGMPCustomLineSolidShape*>(pPrev->shape());
//     IGMPCustomLineSolidShape *pShapeLast = dynamic_cast<IGMPCustomLineSolidShape*>(pLast->shape());
//     if (pShapeLast == nullptr || pShapePrev == nullptr)
//         return;
// 
//     CCurve2dPtr pPrevLine = pShapePrev->line();
//     CCurve2dPtr pLastLine = pShapeLast->line();
// 
//     pPrevLine = ggp::OrientProjectCurve2d(pShapePrev->coordinate(), pPrevLine.get(), CCoordinates3d(), CVector3d::UnitZ);
//     pLastLine = ggp::OrientProjectCurve2d(pShapeLast->coordinate(), pLastLine.get(), CCoordinates3d(), CVector3d::UnitZ);
// 
//     const CVector2d c_StartPrev = pPrevLine->StartPoint();
//     const CVector2d c_EndLast = pLastLine->EndPoint();
//     CLine2d pLine(c_StartPrev, c_EndLast);
// 
//     double dLength = pLine.Length();
//     double dLengthNext = pShapePrev->line()->Length();
//     //处理起点标高变化
//     IGMPElevOperator *const c_pElevOper = pPrev->contnr()->model()->oprCenter()->elevOpr();
//     //承台梁
//     double c_dEndOldValue = 0;
//     double c_dStartOldValue = 0;
// 
//     c_dEndOldValue = pLast->properties()->asInteger(pfnEndPtBottomElev);
//     c_dStartOldValue = pPrev->properties()->asInteger(pfnStartPtBottomElev);
// 
//     double c_dStartNewValue = c_pElevOper->calcElev(pPrev, c_StartPrev, strNewValue);//c_pElevOper->calcElev(pPrev, c_StartPrev, strOldValue);
//     double c_dEndNewValue = c_pElevOper->calcElev(pPrev, c_EndLast, strNewValue);
//     //
//     if (pProp->propName() == pfnStartPtTopElev)
//     {
//         //设置起点标高
//         GMPPropInfoDefaultWriter::writeProp(pPrev->properties()->propByName(pProp->propName()), strNewValue, strOldValue);
//         safeSetProp(pPrev->properties()->propByName(pfnStartPtBottomElev), strNewValue + "-" + QString::number(dStartSectionHeight));
//         calcNormalFDBeamElve(pProp, pPrev, dLength, dLengthNext, c_dEndOldValue, c_dStartNewValue - dStartSectionHeight * 1000);
//     }
//     else if (pProp->propName() == pfnEndPtTopElev)
//     {
//         //设置终点标高
//         GMPPropInfoDefaultWriter::writeProp(pLast->properties()->propByName(pProp->propName()), strNewValue, strOldValue);
//         safeSetProp(pLast->properties()->propByName(pfnEndPtBottomElev), strNewValue + "-" + QString::number(dEndSectionHeight));
//         calcSpecialFDBeamElve(pProp, pPrev, dLength, dLengthNext, c_dStartOldValue, c_dEndNewValue - dEndSectionHeight * 1000);
//     }
//     else if (pProp->propName() == pfnStartPtBottomElev)
//     {
//         //设置起点标高
//         GMPPropInfoDefaultWriter::writeProp(pPrev->properties()->propByName(pProp->propName()), strNewValue, strOldValue);
//         safeSetProp(pPrev->properties()->propByName(pfnStartPtTopElev), strNewValue + "+" + QString::number(dStartSectionHeight));
//         calcNormalFDBeamElve(pProp, pPrev, dLength, dLengthNext, c_dEndOldValue, c_dStartNewValue);
//     }
//     else
//     {
//         //设置起点标高
//         GMPPropInfoDefaultWriter::writeProp(pLast->properties()->propByName(pProp->propName()), strNewValue, strOldValue);
//         safeSetProp(pLast->properties()->propByName(pfnEndPtTopElev), strNewValue + "+" + QString::number(dEndSectionHeight));
//         calcSpecialFDBeamElve(pProp, pPrev, dLength, dLengthNext, c_dStartOldValue, c_dStartNewValue);
//     }
// }
// 
// void GTJArchBeamElevPropInfoWriter::calcNormalFDBeamElve(GMPPropPtr pProp, IGMPElementDrawObj *pEdo,
//     double dLength, double dLengthNext, double dOldElve, double dNewElve)
// {
//     auto safeSetProp = [&] (GMPPropPtr prop, const GString&sValue) {
//         // 单位化
//         GString strTransfer;
//         if (!m_pUnitTransfer || !m_pUnitTransfer->display2Store(prop, sValue, strTransfer, false))
//         {
//             strTransfer = sValue.trimmed();
//         }
//         // 根据数据类型设置值，标高目前只会是gmdtText，因为数据库中对应的是Memo
//         prop->setAsString(strTransfer);
//     };
//     double dSectionHeight = pProp->owner()->asInteger(pfnSectionHeight) / 1000.0;
//     //设置起点标高
//     double dTempElve = 0;
//     double dTemp = dNewElve - dOldElve;
//     if (ggp::IsGreaterEqualThan(dNewElve, 0, g_DistEpsilon))
//     {
//         if (dNewElve < dOldElve)
//         {
//             dTempElve = (dLengthNext) * fabs(dTemp) / dLength;
//             if (dTempElve > dOldElve)
//             {
//                 dTempElve = dOldElve - dTempElve;
//             }
//             else
//             {
//                 dTempElve = dTempElve + dNewElve;
//             }
//         }
//         else
//         {
//             dTempElve = (dLength - dLengthNext)* fabs(dTemp) / dLength;
//             dTempElve += dOldElve;
//         }
//     }
//     else
//     {
//         if (dNewElve < dOldElve)
//         {
//             dTempElve = (dLength - dLengthNext) * fabs(dTemp) / dLength;
//             if (ggp::IsGreaterEqualThan(dOldElve, 0, g_DistEpsilon))
//             {
//                 dTempElve = dOldElve - dTempElve;
//             }
//             else
//             {
//                 dTempElve = dOldElve - dTempElve;
//             }
//         }
//         else
//         {
//             dTempElve = (dLengthNext) * fabs(dTemp) / dLength;
//             dTempElve = -dTempElve + dNewElve;
//         }
//     }
//     //设置起点标高
// 
//     safeSetProp(pEdo->properties()->propByName(pfnEndPtBottomElev), QString::number(dTempElve / 1000));
//     safeSetProp(pEdo->properties()->propByName(pfnEndPtTopElev), QString::number(dTempElve / 1000 + dSectionHeight));
//     //循环计算这些点标高
//     IGMPRelationOperator *pRelaOpr = pEdo->contnr()->model()->oprCenter()->relaOpr();
//     IGMPElementDrawObj *pNext = pRelaOpr->next(pEdo);
//     while (pNext != nullptr)
//     {
//         //起点坐标
//         dSectionHeight = pNext->properties()->asInteger(pfnSectionHeight) / 1000.0;
//         safeSetProp(pNext->properties()->propByName(pfnStartPtBottomElev), QString::number(dTempElve / 1000));
//         safeSetProp(pNext->properties()->propByName(pfnStartPtTopElev), QString::number(dTempElve / 1000 + dSectionHeight));
// 
//         //计算下个坐标点
//         IGMPCustomLineSolidShape *pShapeNext = dynamic_cast<IGMPCustomLineSolidShape*>(pNext->shape());
//         if (pShapeNext == nullptr)
//         {
//             pNext = pRelaOpr->next(pNext);
//             continue;
//         }
//         //根据不同坐标计算不同标高
//         dLengthNext += pShapeNext->line()->ApprLength();
//         if (ggp::IsGreaterEqualThan(dNewElve, 0, g_DistEpsilon))
//         {
//             if (dNewElve < dOldElve)
//             {
//                 dTempElve = (dLengthNext) * fabs(dTemp) / dLength;
//                 if (dTempElve > dOldElve)
//                 {
//                     dTempElve = dOldElve - dTempElve;
//                 }
//                 else
//                 {
//                     dTempElve = dTempElve + dNewElve;
//                 }
//                 if (ggp::isZero(dTempElve, g_DistEpsilon) ||
//                     ggp::IsGreaterEqualThan(dTempElve, dOldElve, g_DistEpsilon))
//                     break;
//             }
//             else
//             {
//                 dTempElve = (dLength - dLengthNext)* fabs(dTemp) / dLength;
//                 dTempElve += dOldElve;
//                 if (ggp::IsLessEqualThan(dTempElve, dOldElve, g_DistEpsilon) ||
//                     ggp::IsGreaterEqualThan(dTempElve, dNewElve, g_DistEpsilon))
//                     break;
//             }
//         }
//         else
//         {
//             if (dNewElve < dOldElve)
//             {
//                 dTempElve = (dLength - dLengthNext) * fabs(dTemp) / dLength;
//                 if (ggp::IsGreaterEqualThan(dOldElve, 0, g_DistEpsilon))
//                 {
//                     dTempElve = dOldElve - dTempElve;
// 
//                 }
//                 else
//                 {
//                     dTempElve = dOldElve - dTempElve;
//                 }
//                 if (ggp::isZero(dTempElve, g_DistEpsilon) || ggp::IsGreaterEqualThan(dTempElve, dOldElve, g_DistEpsilon))
//                     break;
//             }
//             else
//             {
//                 dTempElve = (dLengthNext) * fabs(dTemp) / dLength;
//                 dTempElve = -dTempElve + dNewElve;
//                 if (ggp::IsLessEqualThan(dTempElve, dOldElve, g_DistEpsilon) || ggp::IsGreaterEqualThan(dTempElve, dNewElve, g_DistEpsilon))
//                     break;
//             }
//         }
//         //终点坐标
//         safeSetProp(pNext->properties()->propByName(pfnEndPtBottomElev), QString::number(dTempElve / 1000));
//         safeSetProp(pNext->properties()->propByName(pfnEndPtTopElev), QString::number(dTempElve / 1000 + dSectionHeight));
//         pNext = pRelaOpr->next(pNext);
//     }
// }
// 
// void GTJArchBeamElevPropInfoWriter::calcSpecialFDBeamElve(GMPPropPtr pProp, IGMPElementDrawObj *pEdo, double dLength,
//     double dLengthNext, double dOldElve, double dNewElve)
// {
//     auto safeSetProp = [&] (GMPPropPtr prop, const GString&sValue) {
//         // 单位化
//         GString strTransfer;
//         if (!m_pUnitTransfer || !m_pUnitTransfer->display2Store(prop, sValue, strTransfer, false))
//         {
//             strTransfer = sValue.trimmed();
//         }
//         // 根据数据类型设置值，标高目前只会是gmdtText，因为数据库中对应的是Memo
//         prop->setAsString(strTransfer);
//         //prop->setAsDouble(strTransfer.toDouble() * 1000);
//     };
//     double dSectionHeight = pProp->owner()->asInteger(pfnSectionHeight) / 1000.0;
//     //设置起点标高
//     double dTempElve = 0;
//     double dTemp = dNewElve - dOldElve;
//     if (ggp::IsGreaterEqualThan(dNewElve, 0, g_DistEpsilon))
//     {
//         if (dNewElve > dOldElve)
//         {
//             dTempElve = (dLengthNext) * fabs(dTemp) / dLength;
//             dTempElve = dOldElve + dTempElve;
//         }
//         else
//         {
//             dTempElve = (dLength - dLengthNext)* fabs(dTemp) / dLength;
//             dTempElve += dNewElve;
//         }
//     }
//     else
//     {
//         if (dNewElve > dOldElve)
//         {
//             dTempElve = (dLength - dLengthNext) * fabs(dTemp) / dLength;
//             dTempElve = dNewElve - dTempElve;
//         }
//         else
//         {
//             dTempElve = (dLengthNext) * fabs(dTemp) / dLength;
//             if (ggp::IsGreaterEqualThan(dOldElve, 0, g_DistEpsilon))
//             {
//                 dTempElve = dOldElve - dTempElve;
//             }
//             else
//             {
//                 dTempElve = -dTempElve + dOldElve;
//             }
//         }
//     }
//     //设置起点标高
//     safeSetProp(pEdo->properties()->propByName(pfnEndPtBottomElev), QString::number(dTempElve / 1000));
//     safeSetProp(pEdo->properties()->propByName(pfnEndPtTopElev), QString::number(dTempElve / 1000 + dSectionHeight));
//     //循环计算这些点标高
//     IGMPRelationOperator *pRelaOpr = pEdo->contnr()->model()->oprCenter()->relaOpr();
//     IGMPElementDrawObj *pNext = pRelaOpr->next(pEdo);
//     while (pNext != nullptr)
//     {
//         //起点坐标
//         dSectionHeight = pNext->properties()->asInteger(pfnSectionHeight) / 1000.0;
//         safeSetProp(pNext->properties()->propByName(pfnStartPtBottomElev), QString::number(dTempElve / 1000));
//         safeSetProp(pNext->properties()->propByName(pfnStartPtTopElev), QString::number(dTempElve / 1000 + dSectionHeight));
// 
//         //计算下个坐标点
//         IGMPCustomLineSolidShape *pShapeNext = dynamic_cast<IGMPCustomLineSolidShape*>(pNext->shape());
//         if (pShapeNext == nullptr)
//         {
//             pNext = pRelaOpr->next(pNext);
//             continue;
//         }
//         //根据不同坐标计算不同标高
//         dLengthNext += pShapeNext->line()->ApprLength();
//         if (ggp::IsGreaterEqualThan(dNewElve, 0, g_DistEpsilon))
//         {
//             if (dNewElve > dOldElve)
//             {
//                 dTempElve = (dLengthNext) * fabs(dTemp) / dLength;
//                 dTempElve = dOldElve + dTempElve;
//                 if (ggp::IsLessEqualThan(dTempElve, dOldElve, g_DistEpsilon) ||
//                     ggp::IsGreaterEqualThan(dTempElve, dNewElve, g_DistEpsilon))
//                     break;
//             }
//             else
//             {
//                 dTempElve = (dLength - dLengthNext)* fabs(dTemp) / dLength;
//                 dTempElve += dNewElve;
//                 if (ggp::isZero(dTempElve, g_DistEpsilon) ||
//                     ggp::IsGreaterEqualThan(dTempElve, dOldElve, g_DistEpsilon))
//                     break;
//             }
//         }
//         else
//         {
//             if (dNewElve > dOldElve)
//             {
//                 dTempElve = (dLength - dLengthNext) * fabs(dTemp) / dLength;
//                 dTempElve = dNewElve - dTempElve;
//                 if (ggp::isZero(dTempElve, g_DistEpsilon) ||
//                     ggp::IsLessEqualThan(dTempElve, 0, g_DistEpsilon))
//                     break;
//             }
//             else
//             {
//                 dTempElve = (dLengthNext) * fabs(dTemp) / dLength;
//                 if (ggp::IsGreaterEqualThan(dOldElve, 0, g_DistEpsilon))
//                 {
//                     dTempElve = dOldElve - dTempElve;
//                 }
//                 else
//                 {
//                     dTempElve = -dTempElve + dOldElve;
//                 }
//                 if (ggp::IsLessEqualThan(dTempElve, dNewElve, g_DistEpsilon) ||
//                     ggp::IsGreaterEqualThan(dTempElve, dOldElve, g_DistEpsilon))
//                     break;
//             }
//         }
//         //终点坐标
//         //如果是最后一跨就不修改了，应为之前都已经修改过了
//         if (pRelaOpr->next(pNext))
//         {
//             safeSetProp(pNext->properties()->propByName(pfnEndPtBottomElev), QString::number(dTempElve / 1000));
//             safeSetProp(pNext->properties()->propByName(pfnEndPtTopElev), QString::number(dTempElve / 1000 + dSectionHeight));
//         }
//         pNext = pRelaOpr->next(pNext);
//     }
// }
// 
// void GTJArchBeamElevPropInfoWriter::writeCommonProp(IGMPElementDrawObj *pEdo, GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
// {
//     IGMPRelationOperator *pRelaOpr = pEdo->contnr()->model()->oprCenter()->relaOpr();
//     IGMPElementDrawObj* pPrev = pRelaOpr->first(pEdo);
//     IGMPElementDrawObj* pLast = pRelaOpr->last(pEdo);
// 
//     if (std::wstring::npos != pProp->propName().find(L"Start") && pPrev)
//     {
//         pEdo = pPrev;
//     }
//     else if (std::wstring::npos != pProp->propName().find(L"End") && pLast)
//     {
//         pEdo = pLast;
//     }
//     return GMPPropInfoDefaultWriter::writeProp(pEdo->properties()->propByName(pProp->propName()), strNewValue, strOldValue);
// }

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
GTJFDBeamElevPropInfoWriter::GTJFDBeamElevPropInfoWriter(std::vector<int> *const pAcceptEntTypes)
    : GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.clear();
    m_AcceptProps.push_back(GString::fromStdWString(pfnStartPtTopElev));
    m_AcceptProps.push_back(GString::fromStdWString(pfnEndPtTopElev));
    m_AcceptProps.push_back(GString::fromStdWString(pfnStartPtBottomElev));
    m_AcceptProps.push_back(GString::fromStdWString(pfnEndPtBottomElev));
}
// 判断@pEdo是否为多段折梁。
// @strStartPt 和 @strEndPt 分别为梁的起点和终点的属性名。
// @lengVec用来存来各段梁的长度，非直线梁以两个端点的距离为准
bool GTJFDBeamElevPropInfoWriter::checkMultiSegments(IGMPElementDrawObj *const pEdo, const std::wstring &strStartPt,
    const std::wstring &strEndPt, std::vector<double> &lengthVec)
{
    IGMPRelationOperator *const pRelaOpr = pEdo->contnr()->model()->oprCenter()->relaOpr();
    if ((pRelaOpr->prev(pEdo) == nullptr) && (pRelaOpr->next(pEdo) == nullptr))
    {
        return false;
    }

    auto const getEdoLength = [] (IGMPElementDrawObj *const pCurr, std::vector<double> &lenVec) -> void {
        const CCurve2dPtr pCurve = dynamic_cast<IGMPCustomLineSolidShape *>(pCurr->shape())->line();
        if (pCurve->Type() == Line2dType)
        {
            lenVec.push_back(pCurve->ApprLength());
        }
        else
        {
            lenVec.push_back(pCurve->StartPoint().DistanceTo(pCurve->EndPoint()));
        }
    };

    if (!lengthVec.empty())
    {
        lengthVec.clear();
    }

    auto pEdoCurr = pRelaOpr->first(pEdo);
    getEdoLength(pEdoCurr, lengthVec);
    IGMPProperties *pPropList = pEdoCurr->properties();
    double dStartValue, dEndValue = pPropList->asDouble(strEndPt);
    for (pEdoCurr = pRelaOpr->next(pEdoCurr); pEdoCurr != nullptr; pEdoCurr = pRelaOpr->next(pEdoCurr))
    {
        pPropList = pEdoCurr->properties();
        dStartValue = pPropList->asDouble(strStartPt);
        if (ggp::sameValue(dStartValue, dEndValue, 0.001))
        {
            dEndValue = pPropList->asDouble(strEndPt);
            getEdoLength(pEdoCurr, lengthVec);
        }
        else
        {
            lengthVec.clear();
            return false;
        }
    }
    return true;
}
// 设置多段折梁的标高
// @strStartPt 和 @strEndPt 分别为梁的起点和终点的属性名。
// @dHeight 为整段折梁的起点和终点的标高差
void GTJFDBeamElevPropInfoWriter::configSegmentsElev(IGMPRelationOperator *const pRelaOpr,
    IGMPElementDrawObj *const pEdo, const std::wstring &strStartPt, const std::wstring &strEndPt, const double dHeight,
    const std::vector<double> &lengthVec)
{
    IGMPElementDrawObj *pEdoCurr = pRelaOpr->first(pEdo);
    const GString strBaseElev = pEdoCurr->properties()->asString(strStartPt);
    int nCount = std::accumulate(lengthVec.cbegin(), lengthVec.cend(), 0);
    //modified by yangwf-a:nCount 可能为0，从而导致除零的风险；
    if (nCount == 0)
    {
        return;
    }
    const double dRatio = dHeight / nCount;

    GString strValue;
    QTextStream textStream(&strValue);
    textStream.setNumberFlags(QTextStream::ForceSign);
    double dValue = 0;
    GMPPropPtr pProp = nullptr;
    auto itr = lengthVec.cbegin(), itrEnd = lengthVec.cend();
    if (itr != itrEnd)
    {
        dValue += *itr;
        textStream << dValue * dRatio;
        strValue = strBaseElev + strValue;
        pProp = pEdoCurr->properties()->propByName(strEndPt);
        configProp(pProp, strValue, pProp->asString());
        pEdoCurr = pRelaOpr->next(pEdoCurr);
    }
    for (++itr; itr != itrEnd; ++itr)
    {
        pProp = pEdoCurr->properties()->propByName(strStartPt);
        configProp(pProp, strValue, pProp->asString());
        dValue += *itr;
        strValue.clear();
        textStream << dValue * dRatio;
        strValue = strBaseElev + strValue;
        pProp = pEdoCurr->properties()->propByName(strEndPt);
        pEdoCurr = pRelaOpr->next(pEdoCurr);
        if (pEdoCurr != nullptr)
        {
            configProp(pProp, strValue, pProp->asString());
        }
    }
}
// 将原值为@strOldValue的属性@pProp设置为@strNewValue
void GTJFDBeamElevPropInfoWriter::configProp(GMPPropPtr const pProp, const GString &strNewValue,
    const GString &strOldValue)
{
    GMPPropPtr pCompany;
    auto const pPropList = pProp->owner();
    auto const strName = pProp->propName();
    const GString strHeight = QString::number(pPropList->asDouble(pfnSectionHeight) * 0.001);
    if (strName == pfnStartPtTopElev)
    {
        if (pPropList->hasProp(pfnStartPtBottomElev))
        {
            pCompany = pPropList->propByName(pfnStartPtBottomElev);
            GMPPropInfoDefaultWriter::writeProp(pCompany, strNewValue + " - " + strHeight, pCompany->asString());
        }
    }
    else if (strName == pfnStartPtBottomElev)
    {
        if (pPropList->hasProp(pfnStartPtTopElev))
        {
            pCompany = pPropList->propByName(pfnStartPtTopElev);
            GMPPropInfoDefaultWriter::writeProp(pCompany, strNewValue + " + " + strHeight, pCompany->asString());
        }
    }
    else if (strName == pfnEndPtTopElev)
    {
        if (pPropList->hasProp(pfnEndPtBottomElev))
        {
            pCompany = pPropList->propByName(pfnEndPtBottomElev);
            GMPPropInfoDefaultWriter::writeProp(pCompany, strNewValue + " - " + strHeight, pCompany->asString());
        }
    }
    else if (strName == pfnEndPtBottomElev)
    {
        if (pPropList->hasProp(pfnEndPtTopElev))
        {
            pCompany = pPropList->propByName(pfnEndPtTopElev);
            GMPPropInfoDefaultWriter::writeProp(pCompany, strNewValue + " + " + strHeight, pCompany->asString());
        }
    }
    GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
}

void GTJFDBeamElevPropInfoWriter::writeProp(GMPPropPtr pProp, const GString &strNewValue, const GString &strOldValue)
{
    GString strValue = strNewValue.trimmed();
    if (strValue.isEmpty())
    {
        if (canResumeDefaultValue(pProp, strValue))
        {
            strValue = pProp->schema()->defaultExpr();
        }
        else
        {
            strValue = pProp->schema()->nullExpr();
        }
        strValue.remove('"');
    }
    auto const pPropList = pProp->owner();
    auto const pObject = pPropList->owner();
    if (pObject->type() == IGMPEObject::eoEDO)
    {
        std::wstring strStartPt, strEndPt;
        auto const pEdo = dynamic_cast<IGMPElementDrawObj *>(pObject);
        if (pPropList->hasProp(pfnType) && pPropList->asInteger(pfnType) != 2)
        {
            if (pPropList->hasProp(pfnStartPtTopElev))
            {
                strStartPt = pfnStartPtTopElev;
            }
            if (pPropList->hasProp(pfnEndPtTopElev))
            {
                strEndPt = pfnEndPtTopElev;
            }
        }
        else
        {
            if (pPropList->hasProp(pfnStartPtBottomElev))
            {
                strStartPt = pfnStartPtBottomElev;
            }
            if (pPropList->hasProp(pfnEndPtBottomElev))
            {
                strEndPt = pfnEndPtBottomElev;
            }
        }

        std::vector<double> lengthVec;
        if (checkMultiSegments(pEdo, strStartPt, strEndPt, lengthVec))
        {
            double dHeight;
            auto const strPropName = pProp->propName();
            auto const pOprCenter = pEdo->contnr()->model()->oprCenter();
            auto const pElevOpr = pOprCenter->elevOpr();
            IGMPRelationOperator *const pRelaOpr = pOprCenter->relaOpr();
            if (strPropName.find(L"StartPt") != std::wstring::npos)
            {
                auto pEdoCurr = pRelaOpr->first(pEdo);
                configProp(pEdoCurr->properties()->propByName(strPropName), strValue, strOldValue);
                auto const oPoint = dynamic_cast<IGMPCustomLineSolidShape *>(pEdoCurr->shape())->line()->StartPoint();
                dHeight = pElevOpr->calcElev(pEdoCurr, oPoint, strValue);
                pEdoCurr = pRelaOpr->last(pEdo);
                dHeight = pEdoCurr->properties()->asDouble(strEndPt) - dHeight;
            }
            else
            {
                auto pEdoCurr = pRelaOpr->last(pEdo);
                configProp(pEdoCurr->properties()->propByName(strPropName), strValue, strOldValue);
                auto const oPoint = dynamic_cast<IGMPCustomLineSolidShape *>(pEdoCurr->shape())->line()->EndPoint();
                dHeight = pElevOpr->calcElev(pEdoCurr, oPoint, strValue);
                pEdoCurr = pRelaOpr->first(pEdo);
                dHeight -= pEdoCurr->properties()->asDouble(strStartPt);
            }
            configSegmentsElev(pRelaOpr, pEdo, strStartPt, strEndPt, dHeight * 0.001, lengthVec);
            return;
        }
    }
    configProp(pProp, strValue, strOldValue);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
GTJFDBeamSectionHeightPropInfoWriter::GTJFDBeamSectionHeightPropInfoWriter(std::vector<int> *const pAcceptEntTypes)
    : GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.clear();
    m_AcceptProps.push_back(GString::fromStdWString(pfnSectionHeight));
}

void GTJFDBeamSectionHeightPropInfoWriter::writeProp(GMPPropPtr pProp, const GString &strNewValue,
    const GString &strOldValue)
{
    auto const pPropList = pProp->owner();
    if (pPropList->hasProp(pfnType) && pPropList->asInteger(pfnType) == 2)
    { // 处理承台梁
        GMPPropPtr pCompany;
        auto const pPropList = pProp->owner();
        const GString strSectionHeight = QString::number(pPropList->asDouble(pfnSectionHeight) * 0.001);
        if (pPropList->hasProp(pfnStartPtTopElev))
        {
            pCompany = pPropList->propByName(pfnStartPtTopElev);
            GMPPropInfoDefaultWriter::writeProp(pCompany,
                pPropList->asString(pfnStartPtBottomElev) + " + " + strSectionHeight, pCompany->asString());
        }
        if (pPropList->hasProp(pfnEndPtTopElev))
        {
            pCompany = pPropList->propByName(pfnEndPtTopElev);
            GMPPropInfoDefaultWriter::writeProp(pCompany,
                pPropList->asString(pfnEndPtBottomElev) + " + " + strSectionHeight, pCompany->asString());
        }
    }
    GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
GTJArchBeamSectionHeightWriter::GTJArchBeamSectionHeightWriter(vector<int> * pAcceptEntTypes)
    :GMPPropInfoDefaultWriter(pAcceptEntTypes)
{//AxisOffset
    m_AcceptProps.push_back(GString::fromStdWString(GTJDBCConsts::pfnSectionHeight_BJM));
    //为啥连梁，圈梁的截面高度用的字符和梁不一样啊？
    m_AcceptProps.push_back(GString::fromStdWString(GTJDBCConsts::pfnSectionHeight));

	m_dIsAllPlain = true;
	m_dIsArchToVariable = false;
}

bool GTJArchBeamSectionHeightWriter::validateProp(GMPPropPtr pProp, const GString &strValue, GString &strErrMsg)
{
    IGMPEObject *pObject = pProp->owner()->owner();
    IGMPElement *pElement = nullptr;
    if (!GMPPropInfoDefaultWriter::validateProp(pProp, strValue, strErrMsg))
    {
        return false;
    }
    if (pObject->type() == IGMPEObject::eoEDO)
    {
        IGMPElementDrawObj *pEDO = dynamic_cast<IGMPElementDrawObj *>(pObject);
        pElement = pEDO->element();
    }
    else
    {
        pElement = dynamic_cast<IGMPElement *>(pObject);
    }
    IGMPEdoIterator * pEdoIterator = pElement->createEdoIter(false);
    pEdoIterator->first();
    while (!pEdoIterator->isDone())
    {
        IGMPElementDrawObj *pEDO = pEdoIterator->curItem();
        const auto pShape = dynamic_cast<IGMPSectionLineSolidShape *>(pEDO->shape());
        int oArchType = pShape->archInfoType();
        if (oArchType == laiRingBeam)
        {
            bool oStatus;
            const auto c_pArchShape = dynamic_cast<IGMPRingBeamLineArchInfo *>(pShape->getArchInfo());
            double oRadius = c_pArchShape->radius();
            double oHeight = strValue.toDouble(&oStatus);
            if (std::fabs(oRadius) <= oHeight)
            {
                strErrMsg = strErrMsg.fromLocal8Bit("拱梁截面尺寸过大,不能生成合法图元,请重新输入");
                return false;
            }
        }
        pEdoIterator->next();
    }
    delete pEdoIterator;
    return true;
}

void GTJArchBeamSectionHeightWriter::writeProp(GMPPropPtr pProp, const GString &strNewValue, const GString &strOldValue)
{
    GString newStr = strNewValue.trimmed();
    newStr.remove(' ');
    GStringList strValue = newStr.split('/');

    if (strValue.size() > 0 && !newStr.isEmpty())
    {
        //处理首字母零显示
        int dStartHeight = 0;
        int dEndHeight = 0;
        if (strValue.size() > 1)
        {
            dStartHeight = strValue[0].toInt();
            dEndHeight = strValue[1].toInt();
            newStr = QString::number(dStartHeight) + "/" + QString::number(dEndHeight);
        }
        else
        {
            dStartHeight = strValue[0].toInt();
            newStr = QString::number(dStartHeight);
        }
        newStr.remove(' ');
        strValue.clear();
        strValue = newStr.split('/');
    }
    //全是平梁，直接写入
    if (m_dIsAllPlain)
    {
        return GMPPropInfoDefaultWriter::writeProp(pProp, newStr, strOldValue);
    }
    IGMPEObject *pObject = pProp->owner()->owner();
    //存在拱梁但非变截面
    if (!m_dIsArchToVariable)
    {
        if (pObject->type() == IGMPEObject::eoEDO)
        {
            IGMPElementDrawObj *pLintelEdo = dynamic_cast<IGMPElementDrawObj *>(pObject);
            if (pLintelEdo->elementType() == etLintel)
            {
                pLintelEdo->properties()->setAsBoolean(pfnDoRingBeam, true);
            }
        }
        else
        {
            IGMPElement* pLintelElement = dynamic_cast<IGMPElement *>(pObject);
            if (pLintelElement->elementType() == etLintel)
            {
                IGMPEdoIterator *pLintelEdoIter = pLintelElement->createEdoIter(false);
                pLintelEdoIter->first();
                while (!pLintelEdoIter->isDone())
                {
                    IGMPElementDrawObj *pEDO = pLintelEdoIter->curItem();
                    pEDO->properties()->setAsBoolean(pfnDoRingBeam, true);
                    pLintelEdoIter->next();
                }
                delete pLintelEdoIter;
            }
        }
        if (strValue.size() > 1)
        {
            GString dTemp = pProp->asString();
            return GMPPropInfoDefaultWriter::writeProp(pProp, dTemp, strOldValue);
        }
        else
        {
            return GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
        }
    }
    if (strValue.size() > 1)
    {
        if (strValue[0].toInt() == strValue[1].toInt())
        {
            return GMPPropInfoDefaultWriter::writeProp(pProp, strValue.at(0), strOldValue);
        }
    }

    IGMPElement *pElement = nullptr;
    if (pObject->type() == IGMPEObject::eoENT)
    {
        pElement = dynamic_cast<IGMPElement *>(pObject);
        IGMPEdoIterator *pEdoIter = pElement->createEdoIter(false);
        pEdoIter->first();
        while (!pEdoIter->isDone())
        {
            IGMPElementDrawObj *pEDO = pEdoIter->curItem();
            if (m_dIsArchToVariable)
            {
                IGMPCustomLineSolidShape *pShape = dynamic_cast<IGMPCustomLineSolidShape *>(pEDO->shape());
                pShape->setArchInfoType(laiPlane);
            }
            pEdoIter->next();
        }
        delete pEdoIter;
        GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
    }
    else
    {
        IGMPElementDrawObj *pEDO = dynamic_cast<IGMPElementDrawObj *>(pObject);
        pElement = pEDO->element();
        IGMPEdoIterator *pEdoIter = pElement->createEdoIter(false);
        pEdoIter->first();
        while (!pEdoIter->isDone())
        {
            IGMPElementDrawObj *pEDO = pEdoIter->curItem();
            if (m_dIsArchToVariable)
            {
                IGMPCustomLineSolidShape *pShape = dynamic_cast<IGMPCustomLineSolidShape *>(pEDO->shape());
                pShape->setArchInfoType(laiPlane);
            }
            pEdoIter->next();
        }
        delete pEdoIter;
        GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
    }
}

bool GTJArchBeamSectionHeightWriter::beforeWriteProp(vector<GMPPropPtr>& oProps, const GString& strValue, GString& strErrMsg)
{
    m_dIsAllPlain = true;
    m_dIsArchToVariable = false;

    GString newStr = strValue;
    newStr.remove(' ');
    //判断是否是变截面
    GStringList strNewValue = newStr.split('/');
    if (strNewValue.size() > 1)
    {
        //如 500/500 形式不认为是变截面
        if (strNewValue[0].toInt() == strNewValue[1].toInt())
        {
            return true;
        }
    }
    IGMPElement *pElement = nullptr;
    for (auto it = oProps.begin(); it != oProps.end(); ++it)
    {
        if (!m_dIsAllPlain)
        {
            break;
        }
        GMPPropPtr pProp = *it;
        IGMPEObject *pEObject = pProp->owner()->owner();
        if (pEObject->type() == IGMPEObject::eoENT)
        {
            pElement = dynamic_cast<IGMPElement *>(pEObject);
        }
        else
        {
            IGMPElementDrawObj *pEDO = dynamic_cast<IGMPElementDrawObj *>(pEObject);
            pElement = pEDO->element();
        }

        IGMPEdoIterator *pEdoIter = pElement->createEdoIter(false);
        pEdoIter->first();
        while (!pEdoIter->isDone())
        {
            IGMPElementDrawObj *pEDO = pEdoIter->curItem();
            IGMPCustomLineSolidShape *pShape = dynamic_cast<IGMPCustomLineSolidShape *>(pEDO->shape());
            if (pShape->archInfoType() != laiPlane)
            {
                m_dIsAllPlain = false;
                break;
            }
            pEdoIter->next();
        }
        delete pEdoIter;
    }
    if (!m_dIsAllPlain)
    {
        if (strNewValue.size() > 1)
        {
            const char *const c_sHint =
                "当前构件下存在拱梁/异形拱梁图元,修改为变截面梁后将会失去拱信息,是否继续?";
            if (GMPMessageBox::question(QApplication::activeWindow(),
                qApp->translate(c_szPropInfoWriter, c_Confrim), QString::fromLocal8Bit(c_sHint),
                GlodonMessageBox::No | GlodonMessageBox::Yes, GlodonMessageBox::No) == GlodonMessageBox::Yes)
            {
                m_dIsArchToVariable = true;
            }
        }
    }
    return true;
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
GTJSlabPropWriter::GTJSlabPropWriter(std::vector<int> * pAcceptEntTypes /* = nullptr */)
    :GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.push_back(GString::fromStdWString(pfnTopElev));
    m_AcceptProps.push_back(GString::fromStdWString(GTJDBCConsts::pfnEndConstruct));
}

//斜拱板改标高变平，在这里处理修改标高后拱信息的变化
void GTJSlabPropWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    GString oStrValue = strNewValue.trimmed();
    {
        bool bStatus;
        const double dValue = oStrValue.toDouble(&bStatus);
        if (bStatus)
        {
            if (!ggp::isZero(dValue, g_DistEpsilon))
            {
                oStrValue.setNum(dValue > 0 ? dValue + FLT_EPSILON : dValue - FLT_EPSILON, 'f', 3);
                if (oStrValue.contains(QChar('.')))
                {
                    gtjcommon::trimTrailing(oStrValue, QChar('0'));
                    gtjcommon::trimTrailing(oStrValue, QChar('.'));
                }
            }
        }
    }

    IGMPEObject* pObject = pProp->owner()->owner();
    //如果是构件，走默认的
    if (IGMPEObject::eoENT == pObject->type())
    {
        GMPPropInfoDefaultWriter::writeProp(pProp, oStrValue, strOldValue);
    }
    //如果是图元，在这里更新拱信息
    else
    {
        GMPPropInfoDefaultWriter::writeProp(pProp, oStrValue, strOldValue);
        IGMPElementDrawObj* pParent = dynamic_cast<IGMPElementDrawObj*>(pObject);
        IGMPFaceSolidShape* pParentShape = dynamic_cast<IGMPFaceSolidShape*>(pParent->shape());
        //如果改变现浇板是否竖直边沿，则需要同步天棚的poly，实现天棚随现浇板联动
        if (pProp->propName() == GTJDBCConsts::pfnEndConstruct)
        {
            //由于联动只需要处理点式绘制的天棚
            int nChildCount = pParent->contnr()->model()->oprCenter()->relaOpr()->childrenCount(pParent);
            for (int i = 0; i < nChildCount; i++)
            {
                IGMPElementDrawObj* pChild = pParent->contnr()->model()->oprCenter()->relaOpr()->child(pParent, i);
                IGMPFaceSolidShape *pCeilingShape = nullptr;
                if (pChild != nullptr && pChild->elementType() == etCeiling && pChild->properties()->asBoolean(pfnIsDrawPoint))
                {
                    pCeilingShape = dynamic_cast<IGMPFaceSolidShape*>(pChild->shape());
                    CPolygon* pCeilingPoly = nullptr;
                    if (pParentShape->archInfoType() == faiArch)
                    {
                        pCeilingPoly = GCLCeilingPolyAdjust().processWhenParentIsArchSlab(pParent, pChild);
                    }
                    else if (pParentShape->archInfoType() == faiCone)
                    {
                        pCeilingPoly = GCLCeilingPolyAdjust().processWhenParentIsConeSlab(pParent, pChild);
                    }
                    else if (pParentShape->archInfoType() == faiSphere)
                    {
                        pCeilingPoly = GCLCeilingPolyAdjust().processWhenParentIsSphereSlab(pParent, pChild);
                    }
                    else
                    {
                        pCeilingPoly = pParentShape->poly()->Clone();
                    }
                    pCeilingShape->setPoly(pCeilingPoly);
                    pCeilingPoly->Free();
                }
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
GTJSpiralSlabPropWriter::GTJSpiralSlabPropWriter(vector<int> *const pAcceptEntTypes)
    : GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.clear();
    m_AcceptProps.push_back(GString::fromStdWString(pfnSectionWidth));
}

void GTJSpiralSlabPropWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    IGMPEObject *pObject = pProp->owner()->owner();
    IGMPEObject::EObjectType objectType = pObject->type();
    IGMPElement *pElment = nullptr;
    if (objectType == IGMPEObject::eoENT)
    {
        pElment = dynamic_cast<IGMPElement *>(pObject);
    }
    else
    {
        IGMPElementDrawObj *pEDO = dynamic_cast<IGMPElementDrawObj *>(pObject);
        pElment = pEDO->element();
    }
    const std::wstring &c_wsName = pProp->propName();
    if (c_wsName == pfnSectionWidth)//厚度与螺旋板放射配筋间距联动
    {
        int nPosition = pElment->properties()->asInteger(L"HorzBarPosition");
        int nDistanceToOuterArc = pElment->properties()->asInteger(L"DistanceToOuterArc");
        if (3 == nPosition && strNewValue.toInt() < nDistanceToOuterArc)
        {
            pElment->properties()->setAsInteger(L"DistanceToOuterArc", 0);
        }
    }
    return GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
}

bool GTJSpiralSlabPropWriter::validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg)
{
    return GMPPropInfoDefaultWriter::validateProp(pProp, strValue, strErrMsg);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
GTJRibbonWindowPropWriter::GTJRibbonWindowPropWriter(vector<int> *const pAcceptEntTypes)
    : GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.clear();
    m_AcceptProps.push_back(GString::fromStdWString(pfnAxisOffset));
    m_AcceptProps.push_back(GString::fromStdWString(GTJDBCConsts::pfnFrameThickness));
}

void GTJRibbonWindowPropWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    const std::wstring &c_wsName = pProp->propName();
    if (c_wsName == pfnAxisOffset)//四舍五入保存
    {
        GString ostrValue = strNewValue;
        bool bStatus;
        double dValue = strNewValue.toDouble(&bStatus);
        if (bStatus)
        {
            dValue = Round(dValue);
            ostrValue.setNum(dValue);
        }
        ostrValue = GTJAxisOffsetLineWidthProcessor::calcRealAxisOffsetValue(pProp, ostrValue);
        return GMPPropInfoDefaultWriter::writeProp(pProp, ostrValue, strOldValue);
    }
    else if (c_wsName == GTJDBCConsts::pfnFrameThickness)
    {
    }
    return GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
}

bool GTJRibbonWindowPropWriter::validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg)
{
    const std::wstring &c_wsName = pProp->propName();
    if (c_wsName == GTJDBCConsts::pfnFrameThickness)
    {
        return GMPPropInfoDefaultWriter::validateProp(pProp, strValue, strErrMsg);
    }
    GString strRealValue = GTJAxisOffsetLineWidthProcessor::calcRealAxisOffsetValue(pProp, strValue);
    return GMPPropInfoDefaultWriter::validateProp(pProp, strRealValue, strErrMsg);
}

GTJSumpSectionWidthHeightPropInfoWriter::GTJSumpSectionWidthHeightPropInfoWriter(std::vector<int> *const pAcceptEntTypes)
    : GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.push_back(GString::fromStdWString(pfnSectionWidth));
    m_AcceptProps.push_back(GString::fromStdWString(pfnSectionHeight));
}

GTJSumpSectionWidthHeightPropInfoWriter::~GTJSumpSectionWidthHeightPropInfoWriter()
{
}

void GTJSumpSectionWidthHeightPropInfoWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    //调整集水坑，使其尽量不超出父
    double dNewValue = strNewValue.toDouble();
    double dOldValue = strOldValue.toDouble();
    if (ggp::compareValue(dNewValue, dOldValue, g_DistEpsilon) == vrGreaterThan)
    {
        AdjustSumpCoordinate(pProp, dNewValue, dOldValue);
    }
    //写入属性
    GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
}

bool GTJSumpSectionWidthHeightPropInfoWriter::TheCurveHasParallelsInVectorAndRemoveIt(CCurve2d* pSumpCurve, vector<CCurve2d*>& oSumpCurvesOnParentPoly)
{
    bool bHasParallelCurve = false;
    vector<CCurve2d*>::iterator it;
    for (it = oSumpCurvesOnParentPoly.begin(); it != oSumpCurvesOnParentPoly.end(); ++it)
    {
        CCurve2d* pCurveJ = *it;
        if (GMPPositionFunc2d::isParallel(pSumpCurve, pCurveJ))
        {
            oSumpCurvesOnParentPoly.erase(it);
            bHasParallelCurve = true;
            break;
        }
    }
    return bHasParallelCurve;
}

bool GTJSumpSectionWidthHeightPropInfoWriter::TheLineIsWidth(CLine2d* pLine, CCoordinates3d& oCoord)
{
    CVector2d oDir = pLine->Dir();
    CVector2d oX2d(oCoord.X.X, oCoord.X.Y);
    return oDir.IsParallel(oX2d);
}

bool GTJSumpSectionWidthHeightPropInfoWriter::TheLineIsHeight(CLine2d* pLine, CCoordinates3d& oCoord)
{
    CVector2d oDir = pLine->Dir();
    CVector2d oY2d(oCoord.Y.X, oCoord.Y.Y);
    return oDir.IsParallel(oY2d);
}


void GTJSumpSectionWidthHeightPropInfoWriter::AdjustEDO(IGMPElementDrawObj* pTmpEdo, GMPPropPtr pProp, double dNewValue, double dOldValue)
{
    //TRUE 设置宽  FALSE 设置长
    bool bIsWidthChanged = pProp->propName() == pfnSectionWidth;
    CPositionJudge oPositionJudge;


    if (nullptr == pTmpEdo)
    {
        return;
    }

    IGMPSectionPointSolidShape* pSumpShape = dynamic_cast<IGMPSectionPointSolidShape*> (pTmpEdo->shape());
    if (nullptr == pSumpShape)
    {
        return;
    }

    IGMPElementDrawObj* pTempParentEdo = pTmpEdo->contnr()->model()->oprCenter()->relaOpr()->parent(pTmpEdo);
    if (nullptr == pTempParentEdo)
    {
        return;
    }

    IGMPShape* pParentShape = pTempParentEdo->shape();
    if (nullptr == pParentShape)
    {
        return;
    }

    CPolygonPtr pSumpPoly = pSumpShape->worldPoly();
    if (nullptr == pSumpPoly)
    {
        return;
    }

    CPolygonPtr pParentPoly = pParentShape->worldPoly();
    if (nullptr == pParentPoly)
    {
        return;
    }

    //取集水坑和父重合的边
    vector<CCurve2d*> oSumpCurvesOnParentPoly;
    vector<CCurve2d*> oSumpCurves;
    pSumpPoly->GetCurves(oSumpCurves);
    for (auto itr = oSumpCurves.begin(), itrEnd = oSumpCurves.end(); itr != itrEnd; ++itr)
    {
        EnCurvePolygonPosition position = oPositionJudge.GetCurvePolygonPosition(*itr, pParentPoly.get());
        if (CP_ON == position
            && !TheCurveHasParallelsInVectorAndRemoveIt(*itr, oSumpCurvesOnParentPoly))
        {
            oSumpCurvesOnParentPoly.push_back((*itr)->Clone());
        }
    }

    //通过偏移集水坑局部坐标系来达到不超出父的效果
    CCoordinates3d oCoord = pSumpShape->coordinate();
    vector<CCurve2d*>::iterator it;
    for (it = oSumpCurvesOnParentPoly.begin(); it != oSumpCurvesOnParentPoly.end(); ++it)
    {
        CLine2d* pLine = dynamic_cast<CLine2d*>(*it);
        if (bIsWidthChanged && TheLineIsHeight(pLine, oCoord)
            || !bIsWidthChanged && TheLineIsWidth(pLine, oCoord))
        {
            CVector2d oDir = pLine->Dir();
            oDir.RotateHalfPI(false);
            CCoordinates3d oCoord = pSumpShape->coordinate();
            CVector2d oInsertPoint = CVector2d(oCoord.Origin.X, oCoord.Origin.Y);
            oInsertPoint = oDir*((dNewValue - dOldValue) / 2) + oInsertPoint;
            oCoord.Origin.Set(oInsertPoint, oCoord.Origin.Z);
            pSumpShape->setCoordinate(oCoord);
        }
    }
    for (auto itor = oSumpCurvesOnParentPoly.begin(); itor != oSumpCurvesOnParentPoly.end(); itor++)
    {
        CCurve2d* pCurve = *itor;
        freeAndNil(pCurve);
    }
    oSumpCurvesOnParentPoly.clear();
}

void GTJSumpSectionWidthHeightPropInfoWriter::AdjustSumpCoordinate(GMPPropPtr pProp, double dNewValue, double dOldValue)
{
    if (IGMPElementDrawObj* pEDO = dynamic_cast<IGMPElementDrawObj*>(pProp->owner()->owner()))
    {
        AdjustEDO(pEDO, pProp, dNewValue, dOldValue);
        return;
    }

    if (IGMPElement* pElement = dynamic_cast<IGMPElement*>(pProp->owner()->owner()))
    {
        IGMPEdoIterator * pEdoIter = pElement->createEdoIter();
        IGMPElementDrawObj* pTmpEdo = nullptr;
        for (pEdoIter->first(); !pEdoIter->isDone(); pEdoIter->next())
        {
            pTmpEdo = pEdoIter->curItem();
            AdjustEDO(pTmpEdo, pProp, dNewValue, dOldValue);
        }
    }
}


GTJStripFDUnitPropInfoWriter::GTJStripFDUnitPropInfoWriter(std::vector<int> * pAcceptEntTypes)
    :GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.push_back(GString::fromStdWString(pfnSectionWidth));
    m_AcceptProps.push_back(GString::fromStdWString(pfnThickness));
}

bool GTJStripFDUnitPropInfoWriter::validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg)
{
    return GMPPropInfoDefaultWriter::validateProp(pProp, strValue, strErrMsg);
}

/*
 * @brief    条基属性编辑处理
 * @author
 * @date     2015年12月02日
 */
void GTJStripFDUnitPropInfoWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    Q_ASSERT(pProp);
    double dAxisOffset = 0.0;
    double dTemp = 0.0;
    IGMPEObject * pParentObj = nullptr;
    IGMPEObject * pObj = pProp->owner()->owner();
    if (pObj->elementType() == etTrenchUnit)
    {
        dTemp = 500.0;
    }
    else if (pObj->elementType() == etInsulatingWallUnit)
    {
        dTemp = 200.0;
    }
    else
    {
        dTemp = 1000.0;
    }
    IGMPRelationOperator * RelaOper = pObj->floor()->contnr()->model()->oprCenter()->relaOpr();
    if (pObj->type() == IGMPEObject::eoENT)
    {
        IGMPElement * pElement = dynamic_cast<IGMPElement*>(pObj);
        pParentObj = RelaOper->parent(pElement);
    }
    else
    {
        IGMPElementDrawObj * pEdo = dynamic_cast<IGMPElementDrawObj*>(pObj);
        pParentObj = RelaOper->parent(pEdo);
    }

    if (canResumeDefaultValue(pProp, strNewValue))
    {
        //子恢复默认值
        pProp->initDefaultValue();

        changeEDOAxisOffset(pObj, pParentObj, dAxisOffset, false);

        //默认值1000为条基单元默认宽度
        if (ggp::IsGreaterThan(dAxisOffset, dTemp, getDefaultDistEpsilon()))
        {
            pParentObj->properties()->setIsNull(pfnAxisOffset);
            //父子联动
            changeEDOAxisOffset(pObj, pParentObj, dAxisOffset, true);
        }
        return;
    }

    // 新值为空or空值表达式计算结果,ps ：暂时注释掉（不涉及任何需求）
    //     GString strValue = strNewValue.trimmed();
    //     if (isNullValue(pProp, strValue))
    //     {
    //         pProp->setIsNull();
    // 
    //         changeEDOAxisOffset(pObj, pParentObj);
    //         return;
    //     }

    // 对属性值进行单位化
    GString strTransfer = GTJPropCommon::unitTransferElev(pProp, strNewValue, m_pUnitTransfer);
    //写入属性宽度
    pProp->setAsInteger(strTransfer.toInt());
    pObj->floor()->contnr()->model()->calculate();

    //获得图元距左边线距离
    changeEDOAxisOffset(pObj, pParentObj, dAxisOffset, false);
    double dNewAxisOffset = strTransfer.toDouble() / 2.0;
    if (ggp::IsGreaterThan(dAxisOffset, dNewAxisOffset, getDefaultDistEpsilon()))
    {
        //修改构建属性
        pParentObj->properties()->setIsNull(pfnAxisOffset);
        //父子图元联动
        changeEDOAxisOffset(pObj, pParentObj, dAxisOffset, true);
    }

    if (pObj->type() == IGMPEObject::eoEDO)
    {
        //单元构建父的AxisOffset距离比默认AxisOffset距离大，此时画法不可用，
        //需要对单元构建父的AxisOffset属性值清空，然后采用默认值处理
        IGMPElementDrawObj * pEdo = dynamic_cast<IGMPElementDrawObj*>(pParentObj);
        if (pEdo != nullptr)
        {
            IGMPElement * pElement = pEdo->element();
            double dEntAxisOffset = pElement->properties()->asDouble(pfnAxisOffset);
            double dDefaultAxisOffset = pElement->properties()->asDouble(pfnWidth) / 2;
            if (ggp::IsGreaterThan(dEntAxisOffset, dDefaultAxisOffset, getDefaultDistEpsilon()))
            {
                pElement->properties()->setIsNull(pfnAxisOffset);
            }
        }
    }
}

/*
 * @brief    获取图元左边线距离
 * @author
 * @date     2015年12月02日
 */
void GTJStripFDUnitPropInfoWriter::changeEDOAxisOffset(IGMPEObject *pObj1, IGMPEObject *pObj2, double &dAxisOffset, bool isNull)
{
    //修改构建下所有图元属性
    if (pObj1->type() == IGMPEObject::eoENT)
    {
        IGMPElement *pParentElement = dynamic_cast<IGMPElement*>(pObj2);
        if (pParentElement == nullptr)
        {
            return;
        }
        IGMPEdoIterator *pEdoContnr = pParentElement->createEdoIter(true);
        pEdoContnr->first();
        while (!pEdoContnr->isDone())
        {
            IGMPElementDrawObj *pEDO = pEdoContnr->curItem();
            dAxisOffset = pEDO->properties()->asDouble(pfnAxisOffset);
            if (isNull)
            {
                pEDO->properties()->setIsNull(pfnAxisOffset);
            }
            pEdoContnr->next();
        }
    }

    if (pObj1->type() == IGMPEObject::eoEDO)
    {
        IGMPElementDrawObj * pEdo = dynamic_cast<IGMPElementDrawObj*>(pObj2);
        dAxisOffset = pEdo->properties()->asDouble(pfnAxisOffset);
    }
}

void GTJInsulatingWallThicknessListener::onNotify(int iNotifyType, IGMPProperties* pProps, void* pData, bool bRedoUndo)
{
    if (!bRedoUndo && iNotifyType == ntAfterUpdate)
    {
        IGMPEObject *const pOwner = pProps->owner();
        if (pOwner->elementType() != etInsulatingWall)
        {
            return;
        }
        IGMPPropertySchema *pSchema = reinterpret_cast<IGMPPropertySchema *>(pData);
        if (pSchema->propName() != pfnThickness)
        {
            return;
        }
        IGMPElement* pElement = nullptr;
        if (pOwner->type() == IGMPEObject::eoENT)
        {
            pElement = dynamic_cast<IGMPElement *>(pOwner);
        }
        else
        {
            pElement = dynamic_cast<IGMPElementDrawObj *>(pOwner)->element();
        }
        IGMPEdoIterator* pEDOIter = pElement->createEdoIter(false);
        pEDOIter->first();
        while (!pEDOIter->isDone())
        {
            IGMPElementDrawObj *pEDO = pEDOIter->curItem();
            //modified by zhangh-t 只有在修改过偏轴距的时候，才有必要做下面的事情
            if (pEDO->properties()->hasProp(pfnThickness) 
                && pEDO->properties()->hasProp(pfnAxisOffset) 
                && !pEDO->properties()->isDataNull(pfnAxisOffset))
            {
                double dThiness = pEDO->properties()->asDouble(pfnThickness);
                double dAxisOffset = pEDO->properties()->asDouble(pfnAxisOffset);

                if (ggp::compareValue(dAxisOffset, dThiness, g_DistEpsilon) == vrGreaterThan)
                {
                    pEDO->properties()->setIsNull(GTJDBCConsts::pfnAxisOffset);
                }
            }
            pEDOIter->next();
        }
        delete pEDOIter;
        pEDOIter = nullptr;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////g//////////////////

void GTJTrenchPropListener::onNotify(int iNotifyType, IGMPProperties* pProps, void* pData, bool bRedoUndo)
{
    IGMPElement* pElement = dynamic_cast<IGMPElement*>(pProps->owner());
    if (pElement == nullptr)
    {
        return;
    }
    const int nElementType = pElement->elementType();
    if(bRedoUndo || nElementType != etTrenchUnit)
        return;
    IGMPRelationOperator* pRelaOpr = pElement->contnr()->model()->oprCenter()->relaOpr();
    int nOrdNum = pRelaOpr->ordNum(pElement);
    IGMPDisplayOperator* pDisplayOpr = pElement->contnr()->model()->oprCenter()->displayOpr();
    IGMPEdoIterator* pIter = pElement->createEdoIter(false);
    for (pIter->first(); !pIter->isDone(); pIter->next())
    {
        auto pEDO = pIter->curItem();
        ggp::CSceneNode* pEDONode = pDisplayOpr->getEdoDisplayNode(pEDO);
        if (pEDONode != nullptr)
        {
            GMPEdoGroupNode* pGroupNode = dynamic_cast<GMPEdoGroupNode*>(pEDONode);
            if (pGroupNode)
            {
                int nChildrenCount = pGroupNode->NumChildren();
                for (int i = 0 ; i < nChildrenCount; ++i)
                {
                    CVisualNode* pChildNode = dynamic_cast<CVisualNode*>(pGroupNode->GetChild(i));
                    int nChildTage = pGroupNode->getChildID(pChildNode);
                    if (pChildNode != nullptr && 0x00000001 == nChildTage)
                    {
                        auto pRenderable = pChildNode->GetRenderable(0);
                        if (pRenderable)
                        {
                            pRenderable->SetPriority(pDisplayOpr->getElementOrder(nElementType) - nOrdNum);
                        }
                    }
                }
            }
        }
    }
    //IGMPElementDrawObj* pEdo = NULL;
    //IGMPElement* pEnt = NULL;
    //if (pProps->owner()->type() == IGMPEObject::eoEDO)
    //{
    //    pEdo = dynamic_cast<IGMPElementDrawObj*>(pObject);
    //    m_pService->model()->oprCenter()->displayOpr()->refreshEdoDisplay(pEdo);		
    //}
    //else
    //{
    //    pEnt = dynamic_cast<IGMPElement*>(pObject);

    //    IGMPEdoIterator* pIter = pEnt->createEdoIter();
    //    SCOPE_EXIT{ if (nullptr != pIter) delete pIter; };

    //    for (pIter->first(); !pIter->isDone(); pIter->next())
    //    {
    //        pEnt->contnr()->model()->oprCenter()->displayOpr()->refreshEdoDisplay(pIter->curItem());
    //    }
    //}
}

void GTJTrenchPropListener::onNotify(int iNotifyType, IGMPElement* pEnt, void* pData, bool bRedoUndo)
{
    if (iNotifyType == ntBeforeDelete)
    {
        if (pEnt->elementType() == etTrenchUnit)
        {
            auto iter = m_unitOrdNumMap.find(pEnt);
            if (iter != m_unitOrdNumMap.end())
            {
                m_unitOrdNumMap.erase(iter);
            }
        }
        else if(pEnt->elementType() == etTrench)
        {
            auto iter = m_setOrdNumChangedElements.find(pEnt);
            if (iter != m_setOrdNumChangedElements.end())
            {
                m_setOrdNumChangedElements.erase(iter);
            }
        }
    }
}

void GTJTrenchPropListener::onEndEdit()
{
    
}

//////////////////////////////////////////////////////////////////////////
GTJEmbedBeamPropWriter::GTJEmbedBeamPropWriter(vector<int> *const pAcceptEntTypes)
    : GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.clear();
    m_AcceptProps.push_back(QString::fromStdWString(pfnSectionWidth));
}

void GTJEmbedBeamPropWriter::writeProp(GMPPropPtr pProp, const GString &strNewValue, const GString &strOldValue)
{
    IGMPElement *pElement;
    IGMPEObject *const pObject = pProp->owner()->owner();
    if (pObject->type() == IGMPEObject::eoENT)
    {
        pElement = dynamic_cast<IGMPElement *>(pObject);
    }
    else
    {
        pElement = dynamic_cast<IGMPElementDrawObj *>(pObject)->element();
    }

    double dOffset;
    IGMPProperties *pPropList;
    GString strValue(strNewValue.trimmed());
    bool bStatus;
    const int nValue = strValue.toInt(&bStatus);
    if (pElement->edoCount() != 0)
    {
        const std::unique_ptr<IGMPEdoIterator> edoList(pElement->createEdoIter());
        for (edoList->first(); !edoList->isDone(); edoList->next())
        {
            pPropList = edoList->curItem()->properties();
            if (pPropList->hasProp(pfnSectionWidthExt))
            { // 设置截面宽度的私有属性值
                pPropList->setAsInteger(pfnSectionWidthExt, nValue);
            }
            // 当截面宽度属性小于轴线距左边线距离时，将轴线距左边线距离恢复为默认值
            dOffset = pPropList->asDouble(pfnAxisOffset);
            if (ggp::IsGreaterThan(dOffset, nValue, ggp::g_DistEpsilon))
            {
                pPropList->setIsNull(pfnAxisOffset);
            }
        }
    }
    pPropList = pElement->properties();
    if (strValue.isEmpty())
    {
        pPropList->setIsNull(pfnAxisOffset);
        if (pPropList->hasProp(pfnSectionWidthExt))
        {
            pPropList->setIsNull(pfnSectionWidthExt);
        }
        pProp->setIsNull();
    }
    else
    {
        dOffset = pPropList->asDouble(pfnAxisOffset);
        if (ggp::IsGreaterThan(dOffset, nValue, ggp::g_DistEpsilon))
        {
            pPropList->setIsNull(pfnAxisOffset);
        }
        if (pPropList->hasProp(pfnSectionWidthExt))
        {
            GMPPropInfoDefaultWriter::writeProp(pPropList->propByName(pfnSectionWidthExt), strValue, strOldValue);
        }
        GMPPropInfoDefaultWriter::writeProp(pProp, strValue, strOldValue);
    }
}

bool GTJEmbedBeamPropWriter::validateProp(GMPPropPtr pProp, const GString &strNewValue, GString &strErrMsg)
{
    const GString strValue(strNewValue.trimmed());
    auto const pObject = pProp->owner()->owner();
    if (strValue.isEmpty())
    {
        if (pObject->type() == IGMPEObject::eoEDO)
        {
            return false;
        }
        IGMPElement *const pElement = dynamic_cast<IGMPElement *>(pObject);
        const int nCount = pElement->edoCount();
        return (nCount == 0);
    }
    else
    {
        bool bStatus;
        const int nValue = strValue.toInt(&bStatus);
        if (bStatus && 0 < nValue && nValue < 20001)
        {
            return true;
        }
        strErrMsg = qApp->translate("GTJPropInfoWriter", pEmbedBeamWidthHint);
        return false;
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//直形梯段 
GTJFlightPropWriter::GTJFlightPropWriter(vector<int> *const pAcceptEntTypes) : GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.clear();
    m_AcceptProps.push_back(GString::fromStdWString(GTJDBCConsts::pfnFlightHeight));
    m_AcceptProps.push_back(QString::fromStdWString(GTJDBCConsts::pfnStepHeight));
}

void GTJFlightPropWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    const std::wstring &c_wsName = pProp->propName();
    if (GTJDBCConsts::pfnFlightHeight == c_wsName || GTJDBCConsts::pfnStepHeight == c_wsName)//四舍五入保存
    {
        GString strRealValue = strNewValue;
        bool bStatus;
        double dValue = strRealValue.toDouble(&bStatus);
        if (bStatus)
        {
            dValue = fRound(dValue, 1, 0.001);
            strRealValue.setNum(dValue, 'f', 1);
        }
        return GMPPropInfoDefaultWriter::writeProp(pProp, strRealValue, strOldValue);
    }
    return GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
}

/*
* 名称: validateProp, add for fix GTJ-10294
* @author yanzc, 20160111
* @brief:
* @param[in] pProp --
* @param[in] strValue --
* @param[in] strErrMsg --
* @return
*/
bool GTJFlightPropWriter::validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg)
{
    bool bRes = GMPPropInfoDefaultWriter::validateProp(pProp, strValue, strErrMsg);
    if (!bRes)
    {
        return bRes;
    }
    IGMPEObject *pObject = pProp->owner()->owner();
    IGMPEObject::EObjectType objectType = pObject->type();
    IGMPElement *pElement = nullptr;
    if (objectType == IGMPEObject::eoENT)
    {
        pElement = dynamic_cast<IGMPElement *>(pObject);
    }
    else
    {
        return bRes;
    }
    const std::wstring &c_wsName = pProp->propName();
    if (c_wsName == GTJDBCConsts::pfnStepHeight)
    {
        IGMPEdoIterator* pEdoItr = pElement->contnr()->model()->edoContnr()->createIter(pElement->floor()->iD(), etFlight);
        pEdoItr->first();
        while (!pEdoItr->isDone())
        {
            IGMPElementDrawObj* pItem = pEdoItr->curItem();
            if (pItem->element()->name() == pElement->name())
            {
                double dTotalStepHeight = pItem->properties()->propByName(L"FlightHeight")->asDouble();
                bool bOK = false;
                double dStepHeight = strValue.toDouble(&bOK);
                if (bOK && vrLessThan == ggp::compareValue(dTotalStepHeight, 2 * dStepHeight, g_DistEpsilon))
                {
                    pItem->erase();
                }
            }
            pEdoItr->next();
        }
        delete pEdoItr;
        pEdoItr = nullptr;
    }
    return bRes;
}

//螺旋梯段 
GTJSpiralFlightPropWriter::GTJSpiralFlightPropWriter(vector<int> *const pAcceptEntTypes) : GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.clear();
    m_AcceptProps.push_back(QString::fromStdWString(GTJDBCConsts::pfnStepHeight));
}

void GTJSpiralFlightPropWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    return GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
}

/*
* 名称: validateProp, add for fix GTJ-10294
* @author yanzc, 20160111
* @brief:
* @param[in] pProp --
* @param[in] strValue --
* @param[in] strErrMsg --
* @return
*/
bool GTJSpiralFlightPropWriter::validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg)
{
    bool bRes = GMPPropInfoDefaultWriter::validateProp(pProp, strValue, strErrMsg);
    if (!bRes)
    {
        return bRes;
    }
    IGMPEObject *pObject = pProp->owner()->owner();
    IGMPEObject::EObjectType objectType = pObject->type();
    IGMPElement *pElement = nullptr;
    if (objectType == IGMPEObject::eoENT)
    {
        pElement = dynamic_cast<IGMPElement *>(pObject);
    }
    else
    {
        return bRes;
    }
    const std::wstring &c_wsName = pProp->propName();
    if (c_wsName == GTJDBCConsts::pfnStepHeight)
    {
        IGMPEdoIterator* pEdoItr = pElement->contnr()->model()->edoContnr()->createIter(pElement->floor()->iD(), etSpiralFlight);
        pEdoItr->first();
        while (!pEdoItr->isDone())
        {
            IGMPElementDrawObj* pItem = pEdoItr->curItem();
            if (pItem->element()->name() == pElement->name())
            {
                int nTotalStepHeight = pItem->properties()->propByName(L"TotalStepHeight")->asInteger();
                bool bOK = false;
                int nStepHeight = (int) strValue.toDouble(&bOK);
                if (bOK && nTotalStepHeight < 2 * nStepHeight)
                {
                    pItem->erase();
                }
                //pItem->contnr()->model()->oprCenter()->displayOpr()->refreshEdoDisplay(pItem);
            }
            pEdoItr->next();
        }
        delete pEdoItr;
        pEdoItr = nullptr;
    }
    return bRes;
}

GTJTrenchPropInfoWriter::GTJTrenchPropInfoWriter(IGMPService *const pService, std::vector< int > * pAcceptEntTypes /*= nullptr*/)
    :GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_pService = pService;

    m_AcceptProps.push_back(GString::fromStdWString(pfnBottomElev));
    m_AcceptProps.push_back(GString::fromStdWString(pfnStartPtBottomElev));
    m_AcceptProps.push_back(GString::fromStdWString(pfnEndPtBottomElev));
    m_AcceptProps.push_back(GString::fromStdWString(GTJDBCConsts::pfnSectionTypeID));
}

void GTJTrenchPropInfoWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{

    //调试模式下的断言
    Q_ASSERT(pProp);
    Q_ASSERT(pProp->owner());
    Q_ASSERT(pProp->owner()->hasProp(pfnStartPtBottomElev));

    //该函数把prop表示的属性的值设为sValue
    auto safeSetProp = [&] (GMPPropPtr prop, const GString&sValue) {
        // 单位化
        GString strTransfer = GTJPropCommon::unitTransferElev(prop, sValue, m_pUnitTransfer);
        // 根据数据类型设置值，标高目前只会是gmdtText，因为数据库中对应的是Memo
        prop->setAsString(strTransfer);
    };

    //先做默认值和空值的处理，把地沟的底标高，起点底标高，终点底标高设为层底标高
    const std::wstring&sName = pProp->propName();
    if (strNewValue.trimmed().isEmpty())
    {
        if (sName == pfnStartPtBottomElev)
        {
            //GMPPropPtr StartPtBottomElev = pProp->owner()->propByName(pfnStartPtBottomElev);
            GMPPropPtr EndPtBottomElev = pProp->owner()->propByName(pfnEndPtBottomElev);
            safeSetProp(pProp, qApp->translate(c_strClassName, c_strFloorBottomElev));
            //safeSetProp(StartPtBottomElev, qApp->translate(c_strClassName, c_strFloorBottomElev) );
            safeSetProp(EndPtBottomElev, qApp->translate(c_strClassName, c_strFloorBottomElev));
        }
    }

    //如果底标高不为空，把起点底标高和终点底标高的值设为底标高
    else
    {
        if (sName == pfnStartPtBottomElev)
        {
            //GMPPropPtr StartPtBottomElev = pProp->owner()->propByName(pfnStartPtBottomElev);
            GMPPropPtr EndPtBottomElev = pProp->owner()->propByName(pfnEndPtBottomElev);
            safeSetProp(pProp, strNewValue);
            //safeSetProp(StartPtBottomElev, strNewValue );
            safeSetProp(EndPtBottomElev, strNewValue);
        }
    }
}


/*
* @author		zhangw-n, 20160111
* @brief:		根據屬性，獲取對應的構件
* @param[in]	pProp --
* @return		構件指針
*/
IGMPElement *GTJTrenchPropInfoWriter::getElement(GMPPropPtr pProperty)
{
    IGMPProperties* pProps = pProperty->owner();
    IGMPEObject* pObj = pProps->owner();

    IGMPElement* pElement = nullptr;
    if (pObj->type() == IGMPEObject::eoENT)
    {
        pElement = dynamic_cast<IGMPElement*>(pObj);
    }
    if (pObj->type() == IGMPEObject::eoEDO)
    {
        pElement = dynamic_cast<IGMPElementDrawObj*>(pObj)->element();
    }

    return pElement;
}

/*
* @author		zhangw-n, 20160111
* @brief:		一組屬性接口，包含的section info信息是否和第一個屬性接口的一致
* @param        vector<GMPPropPtr>
* @return		任一個屬性接口的section info有別於第一個屬性，則置 true
*/
bool GTJTrenchPropInfoWriter::isSecInfoPropDiff(const vector<GMPPropPtr>& oProps)
{
    if (oProps.size() < 1) return false;

    QString sProps = oProps[0]->owner()->asString(L"SectionInfo");
    for (int i = 1; i < (int) oProps.size(); i++)
    {
        if (0 != sProps.compare(oProps[i]->owner()->asString(L"SectionInfo")))
        {
            return true;
        }
    }

    return false;
}

/*
* @author		zhangw-n, 20160303
* @brief:		根據屬性，獲取對應的原始構件
* @param[in]	pProp
* @return
*/
void GTJTrenchPropInfoWriter::getOrgElement(const vector<GMPPropPtr>& oProps, set<IGMPElement *>&vOrgElements)
{
    IGMPEObject* pObj = nullptr;
    IGMPElement * pOrgElement = nullptr;

    for (int i = 0; i < (int) oProps.size(); i++)
    {
        pObj = oProps[i]->owner()->owner();

        pOrgElement = nullptr;
        if (pObj->type() == IGMPEObject::eoEDO)
        {
            pOrgElement = dynamic_cast<IGMPElementDrawObj*>(pObj)->element();
        }
        else
        {
            pOrgElement = dynamic_cast<IGMPElement*>(pObj);
        }

        // 選擇多個圖元，原始構件不需要重複記錄
        if (vOrgElements.end() == vOrgElements.find(pOrgElement))
        {
            vOrgElements.insert(pOrgElement);
        }
    }
}

void GTJTrenchPropInfoWriter::onButtonClick(vector<GMPPropPtr>& oProps, bool& bModify)
{
    // 如果是單個圖元被選中，則只有在介面屬性有修改的情況下，才需要做屬性設置
    // 如果是多個圖元被選中，先判斷下个构件的介面屬性是否一致，介面屬性不一致
    // 时，介面設置点击“確認”，构件需要做屬性設置
    bool bForceUpdate = isSecInfoPropDiff(oProps);

    IGMPProperties* pProps = oProps[0]->owner();
    IGMPEObject* pObj = pProps->owner();

    // 截面形狀是共有屬性，這裡根據屬性把對應的構件指針記錄下來方便介面之後設
    // 置屬性。原因有二，見本函數後面注釋
    set<IGMPElement *>vOrgElements;
    getOrgElement(oProps, vOrgElements);

    // 多個對象被選中，做截面形狀調整時，按照第一個圖元（構件）的截面形狀屬性設置初始值
    // 找到第一個構件
    QVector<IGMPElement *>oElements = m_pService->elementListModel(m_pService->elementListGroup())->getCustomElement();

    int nMinOrdNum = INT_MAX;
    int nTmpONum = 0;
    vector<GMPPropPtr>::iterator pItem = oProps.begin();
    while (pItem != oProps.end())
    {
        nTmpONum = oElements.indexOf(getElement(*pItem));
        if (nMinOrdNum > nTmpONum)
        {
            nMinOrdNum = nTmpONum;
            pProps = (*pItem)->owner();
            pObj = pProps->owner();
        }

        pItem++;
    }
    //

    if (nullptr == pObj) return;

    IGMPModel *pModel = nullptr;
    if (pObj->type() == IGMPEObject::eoEDO)
    {
        pModel = dynamic_cast<IGMPElementDrawObj *>(pObj)->contnr()->model();
    }
    else if (pObj->type() == IGMPEObject::eoENT)
    {
        pModel = dynamic_cast<IGMPElement *>(pObj)->contnr()->model();
    }
    else
    {
        return;
    }

    // new 出來的 Widget 指針， 會被 SelParamPolyForm 接走並負責釋放，故此處的 new 不需要 delete 對應
    GTJCustomParamWidgetDefault *pCustomWidget = new GTJCustomParamWidgetDefault();

    QString sProps = pProps->asString(L"SectionInfo");
    GTJSelParamPolyForm selParamPoly(m_pService->model()->paramPolyDB(), pObj, sProps, -1, pCustomWidget, QApplication::activeWindow());

    if (selParamPoly.exec() != QDialog::Accepted) return;

    //
    bModify = (0 != sProps.compare(selParamPoly.selParamValue()));

    // 參數化修改如無變動，則放棄後續的修改
    // 如設置之前屬性有差別，則只要“確認”，就必須設置屬性
    if (bModify || bForceUpdate)
    {
        sProps = selParamPoly.selParamValue();

        GTJParamTrenchSectionInfoParser teSectionParser;

        // 截面形狀屬性被修改了，並且，所有地溝單元都變成無效的了
        if (0 == teSectionParser.validUnitCounts(sProps))
        {
            // 檢測單元有效性，提示	
            if (GMPMessageBox::question(QApplication::activeWindow(),
                qApp->translate(c_szPropInfoWriter, c_Confrim), qApp->translate(c_pGTJPropInfoWriter, c_strInvalidParamTrenchUnits),
                GlodonMessageBox::No | GlodonMessageBox::Yes, GlodonMessageBox::Yes) == GlodonMessageBox::No)
            {
                return;
            }
        }

        // 之所以使用構件做屬性設置，原因有二：
        // 1. 截面形狀是公有有屬性，即使當前用戶選擇的是圖元做界面属性修改，使用對應的構件的屬性器去設置也是可以的
        // 2. 根本原因：選中地溝圖元，在調整了構件的截面形狀之後，地溝單元可能無效，無效的地溝單元將被刪除，對應的圖元也會被刪除。意味著選中的圖
        //    元也會被刪除！為避免後續操作出現在被刪除的無效圖元上做屬性修改，這裡使用圖元對應構件的屬性去寫
        pModel->beginEdit();

        auto oItem = vOrgElements.begin();
        while (oItem != vOrgElements.end())
        {
            (*oItem)->properties()->setAsString(L"SectionInfo", sProps);
            teSectionParser.paramOnUnits((*oItem), m_pService);

            oItem++;
        }

        pModel->endEdit();
    }
}

//------------------------------------------------------------------------------------------------------------------------------------

GTJBeddingSectionHeightPropInfoWriter::GTJBeddingSectionHeightPropInfoWriter(std::vector<int> * pAcceptEntTypes /*= nullptr*/)
    : GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.push_back(GString::fromStdWString(pfnSectionHeight));
}

//dongxd-a 处理垫层宽度修改时同步真实宽度，宽度为默认空时不同步
void GTJBeddingSectionHeightPropInfoWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);

    const double dTemp = strNewValue.toDouble();
    IGMPEObject * pObject = pProp->owner()->owner();
    if (!strNewValue.isEmpty() && !Equals(dTemp, -1.0, g_DistEpsilon))
    {
        if (pObject->type() == IGMPEObject::eoEDO)
        {
            IGMPElementDrawObj * pEdo = dynamic_cast<IGMPElementDrawObj*>(pObject);
            IGMPElement * pElement = pEdo->element();
            //modified by zhangh-t 修改过的构件，不予重复修改
            auto iter = m_setModifiedElements.find(pElement);
            if (iter == m_setModifiedElements.end())
            {
                setPropValue(pElement, strNewValue);
                m_setModifiedElements.insert(pElement);
            }
        }
        else
        {
            IGMPElement * pElement = dynamic_cast<IGMPElement*>(pObject);
            setPropValue(pElement, strNewValue);
        }
    }
    else
    {
        if (pObject->type() == IGMPEObject::eoEDO)
        {
            IGMPElementDrawObj * pEdo = dynamic_cast<IGMPElementDrawObj*>(pObject);
            IGMPElement * pElement = pEdo->element();
            if (pElement->elementSubType() == 220)
            {
                pElement->properties()->setIsNull(L"RealSectionHeight");
            }
        }
        else
        {
            IGMPElement * pElement = dynamic_cast<IGMPElement*>(pObject);
            if (pElement->elementSubType() == 220)
            {
                pElement->properties()->setIsNull(L"RealSectionHeight");
            }
        }
    }
}

void GTJBeddingSectionHeightPropInfoWriter::setPropValue(IGMPElement * pElement, const GString cStrValue)
{
    if (pElement->elementSubType() == 220)
    {
        pElement->properties()->setAsString(L"RealSectionHeight", cStrValue);
        IGMPEdoIterator * pEdoIter = pElement->createEdoIter(true);
        for (pEdoIter->first(); !pEdoIter->isDone(); pEdoIter->next())
        {
            IGMPElementDrawObj * pEdo = pEdoIter->curItem();
            pEdo->properties()->setAsInteger(L"RealSectionHeight", cStrValue.toInt());
            double dAxisOffset = pEdo->properties()->asDouble(pfnAxisOffset);
            if (ggp::IsGreaterThan(dAxisOffset, cStrValue.toDouble(), getDefaultDistEpsilon()))
            {
                pEdo->properties()->setIsNull(pfnAxisOffset);
            }
        }
    }
}

bool GTJBeddingSectionHeightPropInfoWriter::validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg)
{
    IGMPEObject * pObject = pProp->owner()->owner();
    if (pObject->elementSubType() == 220)
    {
        if (!strValue.isEmpty())
        {
            bool oStatus = false;
            const double pSectionHeight = strValue.toDouble(&oStatus);
            if (oStatus)
            {
                if (Equals(pSectionHeight, -1.0, g_DistEpsilon))
                {
                    return true;
                }
                if (pProp->owner()->propByName(pfnAxisOffset)->isDataNull())
                {
                    if (pSectionHeight < 0.00001)
                    {
                        strErrMsg = strErrMsg.fromLocal8Bit("请输入 (0,100000] 之间的整数且 ≥ 轴线距左边线距离");
                        return false;
                    }
                }
                else
                {
                    const double dTemp = pProp->owner()->asDouble(pfnAxisOffset);
                    if (pSectionHeight - dTemp < -0.00001)
                    {
                        strErrMsg = strErrMsg.fromLocal8Bit("请输入 (0,100000] 之间的整数且 ≥ 轴线距左边线距离");
                        return false;
                    }
                }
            }
        }
    }
    return GMPPropInfoDefaultWriter::validateProp(pProp, strValue, strErrMsg);

}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool GTJFootStepInfoWriter::validateProp(GMPPropPtr pProp, const GString &strValue, GString &strErrMsg)
{
    IGMPEObject *pObject = pProp->owner()->owner();
    if (pObject->type() != IGMPEObject::eoEDO)
    {
        return GMPPropInfoDefaultWriter::validateProp(pProp, strValue, strErrMsg);
    }

    if (!GMPPropInfoDefaultWriter::validateProp(pProp, strValue, strErrMsg))
    {
        return false;
    }

    return true;
}

GTJFootStepInfoWriter::GTJFootStepInfoWriter(std::vector<int> * pAcceptEntTypes /*= nullptr*/)
    :GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.push_back(GString::fromStdWString(L"StepCount"));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
GTJBrickWallDFStylePropWriter::GTJBrickWallDFStylePropWriter(vector<int> *const pAcceptEntTypes /*= nullptr*/)
    : GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.push_back(GString::fromStdWString(GTJDBCConsts::pfnBorderColor));
    m_AcceptProps.push_back(GString::fromStdWString(GTJDBCConsts::pfnFillColor));
}

void GTJBrickWallDFStylePropWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    IGMPEObject* pObject = pProp->owner()->owner();
    IGMPElement* pElemnt = NULL;
    IGMPElementDrawObj* pEdo = NULL;
    if (pObject->type() == IGMPEObject::eoENT)
    {
        pElemnt = dynamic_cast<IGMPElement*>(pObject);
    }
    else if (pObject->type() == IGMPEObject::eoEDO)
    {
        pEdo = dynamic_cast<IGMPElementDrawObj*>(pObject);
    }
    //虚墙、填充墙不进行处理直接写入
    if (pElemnt && ((pElemnt->properties()->propByName(pfnType)->asInteger() == gtj::ptVirtualWall) || \
        (pElemnt->properties()->propByName(pfnType)->asInteger() == gtj::ptFillWall)))
    {
        return GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
    }
    else if(pEdo && ((pEdo->properties()->propByName(pfnType)->asInteger() == gtj::ptVirtualWall) || \
        (pEdo->properties()->propByName(pfnType)->asInteger() == gtj::ptFillWall)))
    {
        return GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
    }
    return GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////
GTJStairPropWriter::GTJStairPropWriter(vector<int> *const pAcceptEntTypes /*= nullptr*/)
    : GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.push_back(GString::fromStdWString(GTJDBCConsts::pfnDescription));
}

void GTJStairPropWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    if (pProp->propName() != pfnDescription)
    {
        return;
    }
    if(strNewValue.isEmpty() && (pProp->owner()->owner()->elementType() == etDormer 
        || pProp->owner()->owner()->elementType() == etSwingWin
        || pProp->owner()->owner()->elementType() == etStair))
    {
        return;
    }

    IGMPEObject* pObject = pProp->owner()->owner();
    IGMPElement* pElemnt = NULL;
    IGMPElementDrawObj* pEdo = NULL;
    if (pObject->type() == IGMPEObject::eoENT)
    {
        pElemnt = dynamic_cast<IGMPElement*>(pObject);
    }
    else if (pObject->type() == IGMPEObject::eoEDO)
    {
        pEdo = dynamic_cast<IGMPElementDrawObj*>(pObject);
        pElemnt = pEdo->element();
    }

    //非参数化楼梯不处理
    bool bParamStair = (pElemnt->elementType() == etStair) && (pElemnt->elementSubType() == gtj::subParamStair);
    bParamStair = bParamStair || (pElemnt->elementType() == etDormer && pElemnt->elementSubType() == gtj::subParamDormer);
    bParamStair = bParamStair || (pElemnt->elementType() == etSwingWin && pElemnt->elementSubType() == gtj::subParamSwingWin);
    if (!bParamStair)
    {
        GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
        return;
    }

    LONGLONG llFloorID = -1;
    if (pElemnt->floor())
        llFloorID = pElemnt->floor()->iD();
    IGMPElementIterator* pIter = pElemnt->contnr()->createIter(llFloorID, pElemnt->elementType());
    SCOPE_EXIT { delete pIter; pIter = NULL; };
    pIter->first();
    while (!pIter->isDone())
    {
        IGMPElement* pElement = pIter->curItem();
        if (pElement == nullptr)
        {
            pIter->next();
            continue;
        }

        GString strElement = pElement->name();
        if (strElement != strOldValue)
        {
            pIter->next();
            continue;
        }

        int nOldSize = strOldValue.length();
        pElement->setName(strNewValue);
        IGMPRelationOperator* pRelaOpr = pElement->contnr()->model()->oprCenter()->relaOpr();
        for (int i = 0; i < pRelaOpr->childrenCount(pElement); ++i)
        {
            IGMPElement* pChildElement = pRelaOpr->child(pElement, i);
            if (pChildElement != nullptr)
            {
                QString strChildName = pChildElement->name();
                //根据更改后的构件名称修改子构件中的构件名称并保存
                int nStartPos = strChildName.indexOf("_") + 1;
                strChildName.replace(nStartPos, nOldSize, strNewValue);
                pChildElement->properties()->setAsString(pfnDescription, strChildName, false);
            }
        }
        pIter->next();
    }
}

GTJPostCastElevPropInfoWriter::GTJPostCastElevPropInfoWriter(vector<int> *pAcceptEntTypes /*= nullptr*/)
    : GMPElevPropInfoWriter(pAcceptEntTypes)
{
    m_AcceptProps.push_back(GString::fromStdWString(L"WallOutExpandElev"));  //踏步高度
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

GTJDrawingSetPropInfoWriter::GTJDrawingSetPropInfoWriter(std::vector<int> *const pAcceptEntTypes)
    : GMPPropInfoWriter(pAcceptEntTypes)
{	
    m_AcceptProps.push_back(GString::fromWCharArray(GTJDBCConsts::pfnStdDrawingsName));
    m_AcceptProps.push_back(GString::fromWCharArray(GTJDBCConsts::pfnStdCode));
}

void GTJDrawingSetPropInfoWriter::onButtonClick(std::vector<GMPPropPtr> &propVec, bool &modified)
{
    if (propVec.empty() || propVec.front() == nullptr)
    {
        return;
    }

    GSPModel gspModel = GTJProjFolderGDBManager::getInstance()->getGSP(c_strDrawingSet);
    if (gspModel.isNull())
    {
        return;
    }

    IGMPProperties *pProps = propVec.front()->owner();
    QString strValue = pProps->asString(GTJDBCConsts::pfnStdCode);
    GTJQueryDrawingSetDialog oFrm(gspModel, getElementType(), QApplication::activeWindow(), strValue);

    GTJDialogStyle oStyle;
    oStyle.setMoveResizeEnabled(false);
    oStyle.setWindowFlags(&oFrm, 0);
    oFrm.setStyle(&oStyle);

    if (oFrm.exec() != QDialog::Accepted)
    {
        return;
    }

    GSPTable gspTableChapter = gspModel.findTable(c_strTableDB, c_strTableDrawingElement);
    GString sUpdatedCode = oFrm.normalizeCode(oFrm.getDrawingSetCode());
    if (sUpdatedCode.isEmpty()) return;

    QString strFilter = QString("(%1 = '%2')").arg(c_strFieldCode).arg(sUpdatedCode);

    GSPRecordList listRecords = gspTableChapter.createRecordList(strFilter);
    if (listRecords.count() < 1) return;
    int nEntID = listRecords.records(0).asInteger(c_strFieldID);

    GString sEleTableName = getElementTableName();
    if (sEleTableName.isEmpty()) return;

    GSPTable gspTableElement = gspModel.findTable(c_strTableDB, sEleTableName);
    strFilter = QString("(%1 = %2)").arg(c_strFieldDrawingEntID).arg(nEntID);
    listRecords = gspTableElement.createRecordList(strFilter);
    if (listRecords.count() < 1) return;

    for (auto itr = propVec.cbegin(); itr != propVec.cend(); ++itr)
    {
        pProps = (*itr)->owner();
        // 標準圖集名稱重新設定
        pProps->setAsString(GTJDBCConsts::pfnStdDrawingsName, oFrm.getDrawingSetName());
        // 獲取標準圖代碼重新設定
        pProps->setAsString(GTJDBCConsts::pfnStdCode, oFrm.getDrawingSetCode().toUpper());

        saveProperties(pProps, listRecords.records(0));
    }

    modified = true;
}

void GTJDrawingSetPropInfoWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    Q_ASSERT(pProp);
    Q_ASSERT(pProp->owner());

    pProp->setAsString(strNewValue);	
}

bool GTJDrawingSetPropInfoWriter::validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg)
{
    return true;
}

//////////////////////////////////////////////////////////////////////////
//门
bool GTJDrawingSetDoorPropInfoWriter::saveProperties(IGMPProperties* pProps, GSPRecord pRecord)
{
    if (nullptr == pProps || nullptr == pRecord) return false;
    
    // 洞口寬度    OpeningWidth
    pProps->setAsInteger(pfnOpeningWidth, pRecord.asInteger(GString::fromWCharArray(pfnOpeningWidth)), false);

    // 洞口高度    OpeningHeight
    pProps->setAsInteger(pfnOpeningHeight, pRecord.asInteger(GString::fromWCharArray(pfnOpeningHeight)), false);

    // 框厚        FrameThickness
    pProps->setAsInteger(pfnFrameThickness, pRecord.asInteger(GString::fromWCharArray(pfnFrameThickness)), false);

    // 框左右扣尺寸 FillingWidth
    pProps->setAsInteger(L"FillingWidth", pRecord.asInteger(GString::fromWCharArray(pfnFrameLeftRightDeductSize)), false);

    // 框上下扣尺寸 FillingHeight
    pProps->setAsInteger(L"FillingHeight", pRecord.asInteger(GString::fromWCharArray(pfnFrameTopBottomDeductSize)), false);

    // 框外围面积  FrameArea
    pProps->setAsDouble(pfnFrameArea, pRecord.asFloat(GString::fromWCharArray(pfnFrameArea)) / 1000000, false);
    
    // 洞口面积    OpeningArea
    pProps->setAsInteger(pfnOpeningArea, pRecord.asInteger(GString::fromWCharArray(pfnOpeningArea)), false);

    return true;
}

// 窗
bool GTJDrawingSetWindowPropInfoWriter::saveProperties(IGMPProperties* pProps, GSPRecord pRecord)
{
    if (nullptr == pProps || nullptr == pRecord) return false;

    // 洞口寬度    OpeningWidth
    pProps->setAsInteger(pfnOpeningWidth, pRecord.asInteger(GString::fromWCharArray(pfnOpeningWidth)), false);

    // 洞口高度    OpeningHeight
    pProps->setAsInteger(pfnOpeningHeight, pRecord.asInteger(GString::fromWCharArray(pfnOpeningHeight)), false);

    // 離地高度		AboveFloorHeight
    pProps->setAsInteger(pfnAboveFloorHeight, pRecord.asInteger(GString::fromWCharArray(pfnAboveFloorHeight)), false);

    // 框厚        FrameThickness
    pProps->setAsInteger(pfnFrameThickness, pRecord.asInteger(GString::fromWCharArray(pfnFrameThickness)), false);

    // 框左右扣尺寸 FillingWidth
    pProps->setAsInteger(L"FillingWidth", pRecord.asInteger(GString::fromWCharArray(pfnFrameLeftRightDeductSize)), false);

    // 框上下扣尺寸 FillingHeight
    pProps->setAsInteger(L"FillingHeight", pRecord.asInteger(GString::fromWCharArray(pfnFrameTopBottomDeductSize)), false);

    // 框外围面积  FrameArea
    pProps->setAsDouble(pfnFrameArea, pRecord.asFloat(GString::fromWCharArray(pfnFrameArea)) / 1000000, false);

    // 洞口面积    OpeningArea
    pProps->setAsInteger(pfnOpeningArea, pRecord.asInteger(GString::fromWCharArray(pfnOpeningArea)), false);

    return true;
}

// 门联窗
bool GTJDrawingSetDoorWinPropInfoWriter::saveProperties(IGMPProperties* pProps, GSPRecord pRecord)
{
    if (nullptr == pProps || nullptr == pRecord) return false;

    // 洞口寬度    OpeningWidth
    pProps->setAsInteger(pfnOpeningWidth, pRecord.asInteger(GString::fromWCharArray(pfnOpeningWidth)), false);

    // 洞口高度    OpeningHeight
    pProps->setAsInteger(pfnOpeningHeight, pRecord.asInteger(GString::fromWCharArray(pfnOpeningHeight)), false);

    // 框厚        FrameThickness
    pProps->setAsInteger(pfnFrameThickness, pRecord.asInteger(GString::fromWCharArray(pfnFrameThickness)), false);

    // 窗寬度
    pProps->setAsInteger(GTJDBCConsts::pfnWinWidth, pRecord.asInteger(GString::fromWCharArray(pfnDWWinWidth)), false);

    // 窗離地高度
    //pProps->setAsInteger(GTJDBCConsts::pfnWinAboveFloorHeight, pRecord.asInteger(GString::fromWCharArray(GTJDBCConsts::pfnWinAboveFloorHeight)), false);
    
    // 門框左右扣尺寸
    pProps->setAsInteger(L"DoorFillingWidth", pRecord.asInteger(GString::fromWCharArray(pfnDoorFrameLeftRightDeductSize)), false);

    // 門框上下扣尺寸
    pProps->setAsInteger(L"DoorFillingHeight", pRecord.asInteger(GString::fromWCharArray(pfnDoorFrameTopBottomDeductSize)), false);

    // 窗框左右扣尺寸  WinFillingWidth //WinFrameLeftRightDeductSize
    pProps->setAsInteger(L"WinFillingWidth", pRecord.asInteger(GString::fromWCharArray(pfnWinFrameLeftRightDeductSize)), false);

    // 窗框上下扣尺寸	  WinFillingHeight
    pProps->setAsInteger(L"WinFillingHeight", pRecord.asInteger(GString::fromWCharArray(pfnWinFrameTopBottomDeductSize)), false);

    // 洞口面积    OpeningArea
    pProps->setAsInteger(pfnOpeningArea, pRecord.asInteger(GString::fromWCharArray(pfnOpeningArea)), false);

    // 框外围面积  FrameArea
    pProps->setAsDouble(L"WinFrameArea", pRecord.asFloat(GString::fromWCharArray(pfnFrameArea)) / 1000000, false);
    
    return true;
}

//过梁
bool GTJDrawingSetLintelPropInfoWriter::saveProperties(IGMPProperties* pProps, GSPRecord pRecord)
{
    if (nullptr == pProps || nullptr == pRecord) return false;

    // 混凝土含量    ConcreteUsage
    //pProps->setAsInteger(pfnOpeningWidth, pRecord.asInteger(GString::fromWCharArray(pfnOpeningWidth)));

    // 鋼筋含量    ReinfUsage
    //pProps->setAsInteger(pfnOpeningHeight, pRecord.asInteger(GString::fromWCharArray(pfnOpeningHeight)));

    // 長度       Len ok
    pProps->setAsInteger(pfnLen, pRecord.asInteger(GString::fromWCharArray(pfnLen)), false);

    // 截面寬度	 SectionWidth ok
    pProps->setAsInteger(pfnSectionWidth, pRecord.asInteger(GString::fromWCharArray(pfnSectionWidth)), false);

    // 截面高度	 SectionHeight ok
    pProps->setAsInteger(pfnSectionHeight, pRecord.asInteger(GString::fromWCharArray(pfnSectionHeight)), false);

    // 凸出截面高度	LSectionHeight
    //pProps->setAsInteger(L"FillingHeight", pRecord.asInteger(GString::fromWCharArray(pfnFrameTopBottomDeductSize)));

    // 凸出截面寬度	LSectionWidth
    //pProps->setAsInteger(L"FillingHeight", pRecord.asInteger(GString::fromWCharArray(pfnFrameTopBottomDeductSize)));

    // 體積		Volume  ok
    pProps->setAsDouble(pfnVolume, pRecord.asFloat(GString::fromWCharArray(pfnVolume)), false);

    return true;
}

GTJCustomLineObstacleTagWriter::GTJCustomLineObstacleTagWriter(vector<int> * pAcceptEntTypes /*= nullptr*/)
    :GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.push_back(GString::fromStdWString(L"IsObstacle"));
}

void GTJCustomLineObstacleTagWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    if (pProp->owner()->owner()->type() == IGMPEObject::eoEDO)
    {
        IGMPElementDrawObj* pEDO = dynamic_cast<IGMPElementDrawObj*>(pProp->owner()->owner());
        if ((strOldValue == "0" ||strOldValue.isEmpty()) && strNewValue == "1")
        {
           if (setObstacle(pEDO))
           {
               pEDO->properties()->setAsInteger(pfnTransparent, 30, false);
               GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
           }
        }
        else if (strOldValue == "1" && strNewValue == "0")
        {
            cancleObstatle(pEDO);
            pEDO->properties()->setAsInteger(pfnTransparent, 100, false);
            GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
        }
    }
}

GTJSensorConfigWidget* GTJCustomLineObstacleTagWriter::createSensorInfoWidget()
{
    return new GTJSensorConfigWidget();
}

bool GTJCustomLineObstacleTagWriter::setObstacle(IGMPElementDrawObj* pEDO)
{
    auto setSensorID = [&pEDO] (int64_t nID, FileAddress oAddr, ggp::CDBTable* pTable) {
        ggp::CDBRecord* pRec = pTable->CreateRecordMap(oAddr);
        ggp::CDBField* pIDField = pTable->GetField(L"id");
        ggp::CDBField* pEDOIDField = pTable->GetField(L"edoID");
        ggp::CDBField* pFloorIDField = pTable->GetField(L"floorId");
        bool a = pIDField->SetInt64(pRec, nID);
        a = pEDOIDField->SetInt64(pRec, pEDO->iD());
        a = pFloorIDField->SetInt64(pRec, pEDO->floor()->iD());
        delete pRec;
    };

    static wchar_t* const pfnFireSensorMapping = L"FireSensorMapping";
    GTJSensorConfigWidget* pWidget = createSensorInfoWidget();
    if (pWidget->exec() == QDialog::Accepted)
    {
        IGMPModel* pModel = pEDO->contnr()->model();
        pModel->beginEdit(false);
        ggp::CDatabase* pDB = pModel->projectDB();
        if (pDB)
        {
            ggp::CDBTable* pTable = pDB->GetTable(pfnFireSensorMapping);
            if (pTable)
            {
                FileAddress oAddr;
                if (pTable->Find(pTable->FindField(L"id"), pWidget->m_nSensorID, oAddr))
                {
                    setSensorID(pWidget->m_nSensorID, oAddr, pTable);
                }
                else
                {
                   FileAddress oNewRec = pTable->NewRecord();
                   setSensorID(pWidget->m_nSensorID, oNewRec, pTable);
                   pTable->AddRecord(oNewRec);
                }
            }
        }
        pModel->endEdit();
        return true;
    }
    return false;
}

void GTJCustomLineObstacleTagWriter::cancleObstatle(IGMPElementDrawObj* pEDO)
{
    static wchar_t* const pfnFireSensorMapping = L"FireSensorMapping";
    IGMPModel* pModel = pEDO->contnr()->model();
    pModel->beginEdit(false);
    ggp::CDatabase* pDB = pModel->projectDB();
    if (pDB)
    {
        ggp::CDBTable* pTable = pDB->GetTable(pfnFireSensorMapping);
        if (pTable)
        {
            FileAddress oAddr;
            if (pTable->Find(pTable->FindField(L"edoID"), pEDO->iD(), oAddr))
            {
                pTable->DeleteRecord(oAddr);
            }
        }
    }
    pModel->endEdit();
}
