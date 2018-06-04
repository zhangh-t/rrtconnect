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
static const char* c_strFloorBottomElev = QT_TRANSLATE_NOOP("GTJComplexPropWriter", "��ױ��");

static const char* c_strLinetelHoleBottomElev = QT_TRANSLATE_NOOP("GGJLinetelBeamWriter", "���ڵױ��");
static const char* c_strLinetelHoleTopElevAddBeamHeight = QT_TRANSLATE_NOOP("GGJLinetelBeamWriter", "���ڶ���߼ӹ����߶�");
static const char* c_strLinetelAxisOffset = QT_TRANSLATE_NOOP("GGJLinetelBeamWriter", "��������ȫ��¶��ǽ��,������ (%1,%2) ֮���ʵ��!");
static const char *c_strInvalidNum = QT_TRANSLATE_NOOP("GGJLinetelBeamWriter", "����Ƿ�,%1,����������");
static const char *c_strLinetelPtLenInWall = QT_TRANSLATE_NOOP("GGJLinetelBeamWriter", "�������ཻ,����������");

static const char* c_pGTJPropInfoWriter = "GTJPropInfoWriter";
static const char* c_strSectionWriterType = QT_TRANSLATE_NOOP("GTJPropInfoWriter", "������ (0,50000] ֮�������");
static const char* s_strTips = QT_TRANSLATE_NOOP("GTJPropInfoWriter", "��ʾ");
static const char *c_strElevInvalid = QT_TRANSLATE_NOOP("GTJPropInfoWriter", "����߳�����Χ [-1000,1000]m,����������");
static const char *c_FatherDepthInvalid = QT_TRANSLATE_NOOP("GTJPropInfoWriter", "����Ƿ�,������ (0,100000] ֮�������,��Ȳ�����С�ڶ�����ʪ��������Ҹ���������ֵ���ܴ��� 100000,����������");
static const char* c_strParentElevInvalid = QT_TRANSLATE_NOOP("GTJPropInfoWriter", "��ͼԪ����߳�����Χ [-1000,1000]m,����������");
static const wchar_t   *c_szColumnCapType = L"ColumnCapType";
static const wchar_t   *c_szColumnBaseType = L"ColumnBaseType";
static const char*      c_szDitchEarthInvalidateElev = QT_TRANSLATE_NOOP("GTJPropInfoWriter", "ͼԪ����б�����ò���,ʹ����Ͷ����ཻ��,�����ǲ��ܸ��ڶ����");
static const char* c_strInvalidParamTrenchUnits = QT_TRANSLATE_NOOP("GTJPropInfoWriter", "�װ�,�ǰ�,���Ҳ�ڲ�����Ϊ 0 ʱ��ɾ���Ѿ����ƺõĵع�ͼԪ���Ƿ����?");

static const char *c_szPropInfoWriter = "PropInfoWriter";
static const char *c_Confrim = QT_TRANSLATE_NOOP("PropInfoWriter", "ȷ��");
static const char *c_ArchElevModifyHintInfo = QT_TRANSLATE_NOOP("PropInfoWriter", "�޸ı�ߺ�,�����ߺ���ǰ��߲�ͬ,��ʹ������Ϊƽ����б��,�Ƿ����?");

static const char *const c_strBodyIntersected = QT_TRANSLATE_NOOP("GTJPropInfoWriter", "�������ཻ,����������");

static const char *const pEmbedBeamWidthHint = QT_TRANSLATE_NOOP("GTJPropInfoWriter", "������ (0,20000] ֮�������");

static const wchar_t* pfnFrameLeftRightDeductSize	= L"FrameLeftRightDeductSize";                      //�����ҿ۳ߴ�
static const wchar_t* pfnFrameTopBottomDeductSize	= L"FrameTopBottomDeductSize";                      //�����¿۳ߴ�
static const wchar_t* pfnFrameArea					= L"FrameArea";										//�������e

static const wchar_t* pfnDoorFrameLeftRightDeductSize					= L"DoorFrameLeftRightDeductSize";	 //�T�����ҿ۳ߴ�
static const wchar_t* pfnDoorFrameTopBottomDeductSize					= L"DoorFrameTopBottomDeductSize";	 //�T�����¿۳ߴ�

static const wchar_t* pfnWinFrameLeftRightDeductSize					= L"WinFrameLeftRightDeductSize";	 //�������ҿ۳ߴ�
static const wchar_t* pfnWinFrameTopBottomDeductSize					= L"WinFrameTopBottomDeductSize";	 //�������¿۳ߴ�

static const wchar_t* pfnVolume											= L"Volume";						 //�w�e
static const wchar_t* pfnDWWinWidth										= L"WinWidth";						 //�T��������

GTJParamNAbnormalWriter::GTJParamNAbnormalWriter(ggp::CDatabase* pParamDB, vector<int> * pAcceptEntTypes)
    : GMPParamNAbnormalWriter(pParamDB, pAcceptEntTypes), m_pParamDB(pParamDB)
{
}

void GTJParamNAbnormalWriter::onButtonClick(vector<GMPPropPtr>& oProps, bool& bModify)
{
    SCOPE_EXIT{ afterModifySectionInfo(oProps); };
    //���崦���߼�
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
        //��ģĿǰ��������������û�������ƣ���������һЩ�������������״���ԣ�������û��Poly������ֱ���
        //�˴����Ժ�ģ�������������Կ��ƴ������֮��ɷſ�
        if (pEDO->element()->refType() == ertComplexBody)
        {
            return;
        }
    }
    else if (IGMPElement* pElement = dynamic_cast<IGMPElement*>(pTemp))
    {
        nElementTypeID = pElement->elementType();
        paramPolyDataBase = pElement->contnr()->model()->paramPolyDB();
        //��ģĿǰ��������������û�������ƣ���������һЩ�������������״���ԣ�������û��Poly������ֱ���
        //�˴����Ժ�ģ�������������Կ��ƴ������֮��ɷſ�
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
                //����IDΪ381����ֽ�������������û���������ͼ
                //����IDΪ371����ֽ����δ�����������������
                double dSectionWidth = 0.0;
                if (pSectionInfo->nPolyID == 381 || pSectionInfo->nPolyID == 371)
                {
                    QStringList strList = pSectionInfo->strPolyVal.split(';');
                    //��������Ϊ4�����ٴ�ȷ���������β���ͼ
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

                        // �������������pfnAxisOffset_JZBZ
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
            //���������������⴦��
            if (pPolygon && pPolygon->IsValid() && !pPolygon->IsEmpty())
            {
                //����IDΪ381����ֽ�������������û���������ͼ
                //����IDΪ371����ֽ����δ�����������������
                if (pSectionInfo->nPolyID == 381 || pSectionInfo->nPolyID == 371)
                {
                    QStringList strList = pSectionInfo->strPolyVal.split(';');
                    //��������Ϊ4�����ٴ�ȷ���������β���ͼ
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
                //�ع��������Ƚ����⣬�����⴦��
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
            //�������ͻ�����ʹ��pfnAxisOffset_JZBZ
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
*@brief  ���ν���༭
*@author liuxs-a 2015��1��9��
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

            // ����������Ϣ����������Ĭ��ֵ
            sPolygonData.strHorzSpace = QString("100*10");
            sPolygonData.strVertSpace = QString("100*10");

            // �����Խ���������Ϣ
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

                //xiek 2017��6��14��fixed bug GTJY-17880 ��ʱ����༭�Ĺ������ͣ���poly��ʾ��������������������ʾ
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

        // gaoxu �޸�����ǰ��¼����ֵ��Ϊ���ж��Ƿ��ڶԻ����޸�������
        CPolygon* pTmpPolygon = NULL;
        if (sPolygonData.pPolygon)
        {
            pTmpPolygon = sPolygonData.pPolygon->Clone();
        }
        CVector2d vPoint = sPolygonData.oInsPoint;
        QString tmpHorzSpace = sPolygonData.strHorzSpace;
        QString tmpVertSpace = sPolygonData.strVertSpace;

        oEditorParent.setPolygonData(sPolygonData);

        //xiek 2017��6��21��fixed bug GTJY-18046 ���ò�������޸ĺ���Զ�����
        oEditorParent.getEditPolygonFrm()->setInsertPointModify(true);
        if (oEditorParent.exec() == rtOk)
        {
            modified = false;
            //��д���ݿ�
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

                    //Modified by yangwf��ͼԪ��������Polyʱ���������
                    setPoly(jProperties, sPolygonData.pPolygon);

                    //jiawc 2016��6��14��fixed bug ����޸�������������״��Բ�������α���������
                    GMPPropPtr pSectionWidthProp = jProperties->propByName(pfnSectionWidth);
                    if (NULL != pSectionWidthProp && !pSectionWidthProp->readOnly())
                    {
                        jProperties->setAsInteger(pfnSectionWidth, nOffset);
                    }

                    //lij-af 2015��7��30��fixed bug GMP-7580
                    // ���༭�������Զ������½�����ʱ��Ҳ���ý���������Ϣ
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
    // �ڶ��ϴ�����fixing GTJ-1653
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
            { // ��ʽ���ε��
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
        // �����Խ���������Ϣ
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
    { //��д���ݿ�
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
                //Modified by yangwf��ͼԪ��������Polyʱ���������
                setPoly(pPropVec, polygonData.pPolygon);
                
                //if (pPropVec->hasProp(pfnSectionWidth))
                //jiawc 2016��6��14��fixed bug ����޸�������������״��Բ�������α���������
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
    //��ձ�����Ϣ
    this->clearEdgeInfos(pProperties->owner());
    pProperties->setAsPolygon(pfnSectionPoly, *pPoly);
}
/*!
*@brief  ��ձ�����Ϣ
*@author chengdz 2015��5��8��
*/
void GTJParamNAbnormalWriter::clearEdgeInfos(IGMPEObject* pObj)
{
    //���ͼԪ�ı�����Ϣ
    if (pObj->type() == IGMPEObject::eoEDO)
    {
        IGMPElementDrawObj *pEDO = dynamic_cast<IGMPElementDrawObj*>(pObj);
        //���ͼԪ�ı�����Ϣ
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
    { // ��ǰͼԪ
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
            { // ������������ add by shenc
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
            //�ع��������Ƚ����⣬�����⴦��
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
        { // �������ͻ�����ʹ��pfnAxisOffset_JZBZ
            const double dOffset = pPropList->asDouble(pfnAxisOffset_JZBZ);
            if (ggp::IsGreaterThan(dOffset, dWidth, GMPEpsilonU))
            {
                pPropList->setIsNull(pfnAxisOffset_JZBZ);
            }
        }
    }
}

/*!
*@brief      ��д���ຯ�� fix-TJGJ-27732
*@author     qinyb-a 2015��8��22��
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

    //�жϵ�ǰ�����Ƿ�Ϊ��������������
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
    if (bIsParam) //ֻ�й���������������ʱ��д��
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

    // ���ò����
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
    //�޸Ĺ���������ͼԪ����
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
    //ԭ�� ʼ�ձ��ָ߶Ȳ���
    //����޸Ķ���� ��ͬʱ�޸ĵͱ��
    //����޸ĵױ�� ��ͬʱ�޸Ķ����
    //����޸ĸ߶� ��ͬʱ�޸Ķ����
    Q_ASSERT(pProp);
    Q_ASSERT(pProp->owner());
    Q_ASSERT(pProp->owner()->hasProp(pfnHeight) || pProp->owner()->hasProp(pfnDepth));

    auto safeSetProp = [&] (GMPPropPtr prop, const GString&sValue, bool bCheck) {
        GString strValue = GTJPropCommon::convertElevStr2(sValue);
        GString strTransfer = GTJPropCommon::unitTransferElev(prop, strValue, m_pUnitTransfer);
        // ����������������ֵ�����Ŀǰֻ����gmdtText����Ϊ���ݿ��ж�Ӧ����Memo
        prop->setAsString(strTransfer, bCheck);
    };

    //����Ĭ��ֵ�Ϳ�ֵ�Ĵ���
    // �ָ�Ĭ��ֵ
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
        else if (sName == pfnHeight)//Ŀǰ�������ĸ߶���ֻ���� ����ûʲô�� ��ʱ���Ű�
        {
            pProp->initDefaultValue();
            double dHeight = pProp->owner()->asDouble(pfnHeight) / 1000.0;
            GMPPropPtr pTopElev = pProp->owner()->propByName(pfnTopElev);
            safeSetProp(pTopElev, qApp->translate(c_strClassName, c_strFloorBottomElev) +
                "+" + QString::number(dHeight), true);
        }
        return;
    }


    //�������˼�����Ϊ�߶ȵĵ�λ��mm����ߵĵ�λ��m
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
    else if (sName == pfnHeight)//Ŀǰ�������ĸ߶���ֻ���� ����ûʲô�� ��ʱ���Ű�
    {
        GString sBottomElev = pProp->owner()->asString(pfnBottomElev);
        safeSetProp(pProp->owner()->propByName(pfnTopElev), sBottomElev +
            "+" + QString::number(strNewValue.toDouble() / 1000.0), true);
    }
    //������ʽ�������� FIX BUG TJGJ-28846
    GString strTransfer = GTJPropCommon::unitTransferElev(pProp, strNewValue, m_pUnitTransfer);
    // ����������������ֵ�����Ŀǰֻ����gmdtText����Ϊ���ݿ��ж�Ӧ����Memo
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
        //�������˼�����Ϊ�߶ȵĵ�λ��mm����ߵĵ�λ��m
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
//����
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
    //�����������������
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

//ͼԪ
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
        // BEGIN: FIXED BUG 4078:ѡ������������ʽͼԪ���޷��޸�Ϊͬһ����
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

    //��ȡ�ױ���ַ���
    auto getBottomElevStr = [&] (IGMPProperties* pProps) -> GString {
        GString strBottomElev;
        CCoordinates3d coord;
        IGMPElementDrawObj *const pEDO = dynamic_cast<IGMPElementDrawObj *>(pProps->owner());
        if (pEDO != nullptr)
        {
            coord = pEDO->shape()->coordinate();
        }
        if (!coord.Z.IsParallel(CVector3d::UnitZ, g_DistEpsilon)) // �����б��
        {
            strBottomElev = GString::number(pProps->asDouble(pfnBottomElev) * 0.001);
        }
        else
        {
            strBottomElev = pProps->asString(pfnBottomElev);
        }
        return strBottomElev;
    };

    //�޸ĺ�ȵ�ʱ�������޸Ķ����
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
                //TJGJ-28471  ѡ��ͼԪ���޸�ͼԪ��ȣ��������ҲҪ����
                IGMPProperties *pProps = pElment->properties();
                setTopElev(dThickness, pProps);
            }
            pEDOIterator->first();
            while (!pEDOIterator->isDone())
            {
                IGMPElementDrawObj *pEDO = pEDOIterator->curItem();
                CVector3d vec3dZ = pEDO->shape()->coordinate().Z;
                if (isEqual(vec3dZ.Z, 1.0, ggp::g_DistEpsilon))//ƽ��
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
            //TJGJ-28471  ѡ��ͼԪ���޸�ͼԪ��ȣ��������ҲҪ����
            IGMPProperties *pProps = pElment->properties();
            setTopElev(strNewValue.toDouble(), pProps);
        }

        pEDOIterator->first();
        while (!pEDOIterator->isDone())
        {
            IGMPElementDrawObj *pEDO = pEDOIterator->curItem();
            CVector3d vec3dZ = pEDO->shape()->coordinate().Z;
            if (isEqual(vec3dZ.Z, 1.0, ggp::g_DistEpsilon))//ƽ��
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
    //ת������ַ���
    GString strValue = GTJPropCommon::convertElevStr(sValue);
    // ��λ��
    GString strTransfer = GTJPropCommon::unitTransferElev(prop, strValue, m_pUnitTransfer);
    // ����������������ֵ�����Ŀǰֻ����gmdtText����Ϊ���ݿ��ж�Ӧ����Memo
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
                // ����SectionID
                pProps->setAsInteger(pfnSectionTypeID, esctSectParams);

                // ����PolyID
                pProps->setAsInteger(pfnPolyID, selParamPoly.selPolyID());

                // ����SectionPoly
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

                // ���ò����
                QStringList oInsertPt = selParamPoly.selInsertPoint().split(L',');
                assert(oInsertPt.count() == 2);
                pProps->setAsVector2d(pfnInsertPt, CVector2d(oInsertPt[0].toDouble(), oInsertPt[1].toDouble()));

                // ���ò���ֵ
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
    m_AcceptProps.push_back(GString::fromStdWString(pfnStartPtTopElev)); //��㶥���
    m_AcceptProps.push_back(GString::fromStdWString(pfnEndPtTopElev)); //�յ㶥���
    m_AcceptProps.push_back(GString::fromStdWString(pfnGTJPosition));  //λ��
    m_AcceptProps.push_back(GString::fromStdWString(pfnGTJStartPtLenInWall));  //�������ǽ����
    m_AcceptProps.push_back(GString::fromStdWString(pfnGTJEndPtLenInWall));  //�յ�����ǽ����
    m_AcceptProps.push_back(GString::fromStdWString(pfnAxisOffset));//��ǽƤ�������ߵľ���
    m_AcceptProps.push_back(GString::fromStdWString(pfnSectionWidth));//������
    m_AcceptProps.push_back(GString::fromStdWString(pfnLen));//����
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

        //ѡ��������߲�һ���Ĺ���ʱ��strOldValue�ǣ���������
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

// �ж��޸���/ʼ������ǽ�ľ�����Ƿ񳬹�Բ�ܳ�
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
                //���޸�֮ǰ�Ƕ����·�����պ�Ҫ�ָ��ɶ����Ϸ������ǰ�Ƕ����Ϸ����账��
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
        //�������
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
        //�������
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
        //��������λ��
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
        //�������
        safeSetProp(pProp->owner()->propByName(pfnStartPtTopElev), strNewValue);

        //��������λ��
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

    // ���´��������ǽ����
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
            pLine->Extend(nChangeValue, false);  //���ķ����ӳ�
        else
            pLine->SetStartT(pLine->StartT() - nChangeValue);  //������������

        //���ù�������
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
            pLine->Extend(nChangeValue, true);  //�յ�������ӳ�
        else
            pLine->SetEndT(pLine->EndT() + nChangeValue);  //�յ�ķ�������

        //���ù�������
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
        //ȡ�ù�����ͼԪ�Ķ��ڳ���
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
        //ֻ���ǳ��ȼ�С��������������ǽ�ڳ�������ֵ�����
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
    //����ߣ�λ���������Ϊ�յĻ��������쳣��ֱ�ӷ���֮ǰ��ֵ
    if (canResumeDefaultValue(pProp, strValue))
    {
        return true;
    }

    //modify by liujx Fix.bug TJGJ-20378 �� TJGJ-19981
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
        if (strValue.isEmpty())//ֱ����յĻ���������Ĭ�ϻָ�ΪĬ��ֵ
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
        //���ڳ�����Χ�Ĺ��������߾���ǽƤ��������룬�ж�ʱ�����ȼ���������ǽ>����ǽ>����ǽ>���壬ͬһ��������ID��С����
        IGMPElementDrawObj *pEDO = dynamic_cast<IGMPElementDrawObj *>(pProp->owner()->owner());
        IGMPModel *pModel = pEDO->contnr()->model();
        IGMPElementDrawObj *pParentEdo = pModel->oprCenter()->relaOpr()->parent(pEDO);
        //�����ȼ�������˳��
        vector<IGMPElementDrawObj *> oWallVector;
        getWallByLintelParent(oWallVector, pEDO, pParentEdo);
        const int nParentELement = pParentEdo->elementType();
        //�������ĸ�Ϊ�Ŵ������ж��Ƿ񳬳�����үү�ķ�Χ��������ֱ����ʾ������Ϣ
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
            //�����Ѿ��жϹ��������Ŵ�����ʱ������үү���˴�����Ҫ���ж�
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
    vector<IGMPElementDrawObj *> oWall1;            //����ǽ
    vector<IGMPElementDrawObj *> oWall2;            //����ǽ
    vector<IGMPElementDrawObj *> oInsulatingWall;   //����ǽ
    vector<IGMPElementDrawObj *> oParapet;          //����
    for (pEDOIterator->first(); !pEDOIterator->isDone();)
    {
        //���ڴ��ʹ���box�Ƚϴ���˵���������ЩͼԪ�����ǲ���Ҫ�ģ��˴�ͨ�����ཻ���˵����õ�ǽͼԪ
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
    //�Ը��������ٽ���id��С���������
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
            // ��Է��½ǶȽ������⴦��
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
    //���ӷ��½Ƕ������������뱣����λС��
    auto const pPropName = pProp->propName();
    if (0 == pPropName.compare(GTJDBCConsts::pfnAngle))
    {
        if (!strNewValue.isEmpty())
        {
            strTmpValue = QString::number(Round(strTmpValue.toDouble() * 1000) * 0.001, 'f', 3);
        }
    }

    // ����ֵû�ı�ʱ��������
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

    // ���ø���ӿڣ�д������
    GMPPropInfoDefaultWriter::writeProp(pProp, strTmpValue, strOldValue);
    IGMPElementDrawObj *pSumpEDO = dynamic_cast<IGMPElementDrawObj*>(pProp->owner()->owner());
    // ��ǰ�༭�������Ǽ�ˮ��ͼԪ�����ԣ���Ҫͬʱ����ͼԪ�ı�����Ϣ
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

    // �Ѿ���writeProp�����ڣ������˸����ͬ���麯�����޸ĵ�����д����ͼԪ
    // ��ʱ��ȡͼԪ�ı�����Ϣ��ȡ�õı�����Ϣ�����µ�
    GString strEDOVar,
        strEDOOutWidth = pSumpEDO->properties()->asString(GTJDBCConsts::pfnOutside);
    int nSlopeInput = pSumpEDO->properties()->asInteger(GTJDBCConsts::pfnSlopeInput);
    switch (nSlopeInput)
    {
        // ���½Ƕ�
        case 0:
            strEDOVar = pSumpEDO->properties()->asString(GTJDBCConsts::pfnAngle);
            break;

            // ���µ׿�
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
        // ��ˮ��worldpolyֻ��һ����������getItem�ڶ�������Ϊ0
        // ���Ѿ����ñ�����Ϣʱ�����±�����Ϣ
        pEdgeInfo = pEdgeInfos->getItem(i, 0);
        if (pEdgeInfo)
        {
            GStringList strListParam = pEdgeInfo->paramStr().split(';');
            // ��ˮ�ӱ�����Ϣֻ�����������߾���--���µ׿�/���½Ƕ�
            assert(strListParam.size() == 2);

            // ��ǰ�޸ĵ��ǳ��߾���
            if (strPropName == wstring(GTJDBCConsts::pfnOutside))
            {
                // ������ͬ��Ϣд�룬���ݿ�ı䣨��ʹ����ͬ����Ϣ���ᴥ������
                if (strEDOOutWidth != strListParam[0])
                {
                    pEdgeInfo->setParamStr(strEDOOutWidth + ";" + strListParam[1]);
                }
            }
            // ��ǰ�޸ĵ��Ƿ��µ׿�/���½Ƕ�
            // ��ǰ�л�������ģʽ
            else if (((strPropName.compare(GTJDBCConsts::pfnAngle) == 0)
                || (strPropName.compare(GTJDBCConsts::pfnBottomWide) == 0))
                || (strPropName.compare(GTJDBCConsts::pfnSlopeInput) == 0))
            {
                // ������ͬ��Ϣд�룬���ݿ�ı䣨��ʹ����ͬ����Ϣ���ᴥ������
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
            // ���û�б�����Ϣ����Ĭ�ϵ���Ϣ
            pEdgeInfos->insertEdge(i, eitEdgeSlope, &pEdgeInfo, 0);
            if (pEdgeInfo)
            {
                // ��ǰ�޸ĵ��ǳ��߾���
                QString strWriteOutWidth("500");
                if (strPropName.compare(GTJDBCConsts::pfnOutside) == 0)
                {
                    strWriteOutWidth = strEDOOutWidth;
                }
                QString strParams = strWriteOutWidth + ";" + strEDOVar;
                pEdgeInfo->setParamStr(strParams);
            }
        }
    }// end for----worldpoly�ı�
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

////////////////////////////////////////////�����������ֵ��////////////////////////////////////////////
/*!
*@brief    ����
*@author
*@param[in]vector<int> * pAcceptEntTypes       �ɴ���Ĺ�������, NULLָ�������� ,GMPPropInfoBuilder���������
���������������
*@return
*/
GTJSectionWidthWriter::GTJSectionWidthWriter(vector<int> * pAcceptEntTypes)
    :GMPSectionWidthWriter(pAcceptEntTypes)
{
    //m_AcceptProps.push_back(GString::fromStdWString(pfnSectionWidth));
}

/*!
*@brief     д������������
*@author    qinyb 2015��01��21��
*@param[in] pProp--����
*@param[in] strNewValue--��ֵ
*@param[in] strOldValue--��һ��ֵ
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

////////////////////////////////////////////�����������ֵ��////////////////////////////////////////////
/*!
*@brief    ����
*@author   qinyb
*@param[in]vector<int> * pAcceptEntTypes       �ɴ���Ĺ�������, NULLָ�������� ,GMPPropInfoBuilder������������������������
*@return
*/
GTJSectionHeightWriter::GTJSectionHeightWriter(vector<int> * pAcceptEntTypes /*= nullptr*/)
    :GMPSectionHeightWriter(pAcceptEntTypes)
{
    //m_AcceptProps.push_back(GString::fromStdWString(pfnSectionHeight));
}

/*!
*@brief     д����ߡ��������
*@author    qinyb 2015��01��21��
*@param[in] pProp--����
*@param[in] strNewValue--��ֵ
*@param[in] strOldValue--��һ��ֵ
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
    //���崦���߼�
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
            // ����������Ϣ����������Ĭ��ֵ
            sPolygonData.strHorzSpace = "100*10";
            sPolygonData.strVertSpace = "100*10";

            // �����Խ���������Ϣ
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
            // ����������Ϣ����������Ĭ��ֵ
            sPolygonData.strHorzSpace = "20*15";
            sPolygonData.strVertSpace = "100*11";

            // �����Խ���������Ϣ
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

    //���˷���û�в�������ԣ���������Ĭ������Ϊԭ��
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
*@brief    ������������
*@author   zhangh-t
*@date     2015��3��27��
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
        //�ж�Depth�Ƿ�Ϊ����������Ƿ�����������һ������������Ƿ�
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
    strValue.replace(QChar('��'), ':');

    // BEGIN: MODIFY BY 2015-05-22 �޸�bugTJGJ-18974:���롰1������1/������
    if (strValue == strDiv || strValue == strRatio)
    {
        m_bSuc = false;
        return strValue;
    }
    // END

    //ƥ�䣺��1:���� "1/"
    if (strValue.startsWith(strDiv) || strValue.startsWith(strRatio))
    {
        strValue = strValue.remove(0, 2);//ȥ�����С��� "1/"
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
    //ɾ����Ч0
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
*@brief    �������Ժ������������
*@author   yangwl-a
*@date     2015��4��2��
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
    //����ҵ����
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
    //��GCLWallPropListener�д���
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

        /*�� �����ͬ*/
        int nEmbedWidth = pEmbedEdo->properties()->asInteger(pfnSectionWidthExt);
        if (nWallThickness != nEmbedWidth)
        {
            pEdoIter->next();
            continue;
        }

        /*�� ������һ��*/
        auto pEmbedShape = dynamic_cast<IGMPCustomLineSolidShape *>(pEmbedEdo->shape());
        CCurve2dPtr pEmbedCurve = pEmbedShape->worldLine();
        if (pWallCurve->Type() != pEmbedCurve->Type())
        {
            pEdoIter->next();
            continue;
        }
        bool bIsSameDir = false;
        /*�� ֱ��ƽ��*/
        if (pWallCurve->Type() == Line2dType)
        {
            const double c_dParallelEpsilon = 2E-3;  //ƽ�е���󻡶�
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
            /*�� ����Բ��,�뾶��ͬ*/
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

        /*�� body�ཻ*/
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
            //��������Ǿ���ģ�����ܾ���ȡ��
            if (pEmbedShape->isMirror())
            {
                bReveres = !bReveres;
            }
            //���ǽ�Ǿ���ģ�����ܾ���ȡ��
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
    //added by zhangh-t һЩ�������͵�����û�б�Ҫ�����渴�ӵĹ���
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

    //���ھ��������߾�����߾�����ʾ��ֵ�Ǽ��������ʵֵ�������޸ĺ�Ȼᵼ�����߾�����߾���仯������������Ҫ����һ�£�д��ȥ��
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
                //zhangh-t fixed bug TJGJ-29533 ���δ����⴦��
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
                                // �������������pfnAxisOffset_JZBZ
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
                    // �������������pfnAxisOffset_JZBZ
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
                        // �������������pfnAxisOffset_JZBZ
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
                    //��GCLWallPropertyListener�д���
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

    //д������
    GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
    if (pObject->type() == IGMPEObject::eoENT)
    {
        if (!isNeedChangeAxialOffset(nElmType))
        {
            changeLintelAxisOffset(pProp);
            return;
        }
        //��������������
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
    //��Դ��ڴ����ԵĹ������д���
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
                    { //���ֽ�ҵ������
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
                        { //���ֽ�ҵ������
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
        //������ͼԪ��������߾�������߾�������
        GMPPropInfoDefaultWriter::writeProp(pProp, strNewValue, strOldValue);
        pEDO->contnr()->model()->calculate();

        IGMPProperties *const pPropList = pProp->owner();
        //����������Ե�����
        changeLintelAxisOffset(pProp);
        //��������������
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
    //��������ȡ��ǰ������ͼԪ������һ�ж�
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
    //˽������ȡ��ǰͼԪ�����ж�
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
    //���޸Ĵ��ʹ����Ĵ���
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
        //�����������޸�ǽ�ĺ�ȵĴ���
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
        //���ĺ�����Ժ�ƫ�ľ�û�й�ϵ
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
    // ������ֵ���е�λ��
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
*@brief     ����/�ֽ���������ǽ�������
*@author    liuk-g 2015��08��04��
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
    �������汾�У��������ThicknessΪ�������ԣ����ڸֽ�汾�У���Ϊ˽������
    ���,�����������ͼԪ�ķ�ʽ��ͬ������Ҳ�Ͳ�ͬ.
    */
    if (!isPublic)
    {
        //2016-12-1 ��ʱ�ļ���ǽ/����ǽ�ĺ��/ƫ�ľ����Ծ�Ϊ����
        return onNotifyByGGJ(iNotifyType, pProps, pData, bRedoUndo);
    }
    if (pSchema->propName() != pfnThickness)
    {
        return;
    }
    IGMPElement *pElement = nullptr;
    //�������ԣ�ͨ�������������ͼԪ
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
    //������������߾��룬PS������֪��
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
    //����ǽ����֮������
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
        //ֻ��ע�������Եı仯
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
    //�޸�ǽ�ĺ�ȣ���Ҫͬ�������ĺ��
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
                //�߿�������Ҫ����
                if (nType == 1)
                {
                    pedoIterator->next();
                    continue;
                }
            }
            //¥���Ƿ���ͬ
            if (pSideBeamEdo->floor() != pEDO->floor())
            {
                pedoIterator->next();
                continue;
            }
            //box��һ��
            if (2 == nFlag && !oEDOBox.IsIntersect(pSideBeamEdo->shape()->box()))
            {
                pedoIterator->next();
                continue;
            }
            //�ж�������ǽ���Ƿ�ƽ��
            IGMPCustomLineSolidShape *pWallShape = dynamic_cast<IGMPCustomLineSolidShape*>(pFirst->shape());
            IGMPCustomLineSolidShape *pSideBeamShape = dynamic_cast<IGMPCustomLineSolidShape*>(pSideBeamEdo->shape());
            CCurve2dPtr pWallLine = pWallShape->worldLine();
            CCurve2dPtr pSideBeamLine = pSideBeamShape->worldLine();
            
            /*����ֽ�����԰���ϵ��BUG������ԭ����
            1������һ�µ�����ǽ����
            2������ͬ�����ǳ�������Ҳ��Ҫ����
            3�����ǽ����ֻ������ǰ��ѡ���ͼԪ
            4�����뿼�Ǿ���֮��������������ʽ��������*/
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
            //�жϽ������Ƿ�һ��
            if (sPropName == pfnThickness && nSideBeamWidth == m_iSectionWith)
            {
                linkage.nLinkageType = 0;
                m_vecLinkage.push_back(linkage);
            }
            //���������߾���  modify by liuk-g GTJY-16760 �������޸Ĺ�axisoffset�İ�/������������ǽ����
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
        //�ڲ�ѭ������Ҫ��ײbox
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
            //�ڲ�ѭ����Ҫ��ײbox������
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
        //�ж�������ǽ���Ƿ�ƽ��
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
                //�����ͬ��������ܾ���ȡ��
                if (!GMPPositionFunc2d::isSameDirection(pWallLine.get(), pSideBeamLine.get()))
                {
                    bReveres = !bReveres;
                }
                //��������Ǿ���ģ�����ܾ���ȡ��
                if (pSideBeamShape->isMirror())
                {
                    bReveres = !bReveres;
                }
                //���ǽ�Ǿ���ģ�����ܾ���ȡ��
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
                //�����ͬ��������ܾ���ȡ��
                if (!GMPPositionFunc2d::isSameDirection(pWallLine.get(), pSideBeamLine.get()))
                {
                    bReveres = !bReveres;
                }
                //��������Ǿ���ģ�����ܾ���ȡ��
                if (pSideBeamShape->isMirror())
                {
                    bReveres = !bReveres;
                }
                //���ǽ�Ǿ���ģ�����ܾ���ȡ��
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



/////////////////////////////////////////������״���㰴ť///////////////////////////////////////////////////////////////////////////////
GTJParamSectionPropInfoWriter::GTJParamSectionPropInfoWriter(std::vector<int> *const pAcceptEntTypes)
    : GTJParamNAbnormalWriter(nullptr, pAcceptEntTypes)
{
    m_AcceptProps.clear();
    m_AcceptProps.push_back(QString::fromStdWString(pfnSectionTypeID));
}

//����ֵУ��
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

//������״���㰴ť����¼�����
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
    { // ���������������ν���༭
        editAbnormitySection(elmType, propVec, modified);
    }
    else if (esctSectParams == nSectionTypeID)
    { // ��������
        ggp::CDatabase *const paramPolyDataBase = pModel->paramPolyDB();
        //new ������ Widget ָ�룬 �ᱻ SelParamPolyForm ���߲������ͷţ��˴��� new ����Ҫ delete ��Ӧ
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
            //�жϵ�ǰ�����Ƿ�Ϊ��������������
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
                //fix.bug��GTJY-5599 ��������/�޸Ĳ�����ͼԪ�Ľ�����״�еĳߴ磬�������
                pPropList = (*itr)->owner();
                if (parametric) //ֻ�й���������������ʱ��д��
                {
                    pPropList->setAsString(GTJDBCConsts::pfnSectionInfo, strSectionInfo);
                }
                // ����SectionID
                pPropList->setAsInteger(pfnSectionTypeID, esctSectParams);

                //fixed bug GTJY-5819 ��ô������Ϊ�˴������ݿ�仯����bug�����ĳ����ﹹ�첻��������
                pPropList->setAsInteger(pfnPolyID, -1);
                // ����PolyID
                pPropList->setAsInteger(pfnPolyID, pSectionInfo->nPolyID);
                // ���ò����
                pPropList->setAsVector2d(pfnInsertPt, insertPoint);
                // ���ò���ֵ
                pPropList->setAsString(pfnPolyValue, pSectionInfo->strPolyVal);
                // ����SectionPoly
                dWidth = 0;
                if (pPolygon)
                { //fixed bug TJGJ-31176 ǽ/Բ�β�����ǽ�޸Ľ�����״Ϊ�Ƿ���ͼԪ���������
                    const CBox2d polyBox = pPolygon->Box();
                    dWidth = polyBox.MaxPt().X - polyBox.MinPt().X;
                    if (_finite(dWidth) && dWidth > 1)
                    {
                        pPolygon->MergeCoedges(); // GTJY-14862 ����ʹ������ĺ����������д�bug
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
        { // �˴��ǲο������ӵĵױ�������нű����߼�����Ϊ��δ����ǰ����֪�����ĸ��ӿڿ��Ի�֪�ű�ִ�к����Ե�ֵ�Ƕ��١�
            if (pEdo->floor()->code().compare("0") == 0)//������codeΪ0
            {
                strValue = QString::fromLocal8Bit("�����ױ��");
            }
            else
            {
                strValue = QString::fromLocal8Bit("��ױ��");
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
    �µĶ��������޸�ԭ��
        ��ÿ����ͼԪб����֪���ο�б�����ȡ�׶Σ��յ�ȡβ��
        �ڵ�һ��������/β��body�������죬ʣ��б��һ�µ���body�Ƿ���������body�غ�
        ���Ե�һ������/�յ��ߣ��������������һ������/����ߵı�߲��һ�������ö������ȣ�Line�ĳ���֮�ͣ�
    Ϊ���ݣ��õ��µ�б��Slope��
        �ܵ���ÿ��������������ͼԪ����б�ʡ�
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
            strErrMsg = QString::fromLocal8Bit("��ߴ���");
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
    // 00/�� 01/��� 10/�ն� 11/�յ�
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
                    //                     //Ϊ��ȡ��Ĭ��ֵ���Ƚ�Ĭ��ֵд�����ݿ⣬���㣻ȡ��Ĭ��ֵ���ٽ�֮ǰ��ֵд��ȥ�����㡣����
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
                                "�޸ı�ߺ������ߺ�֮ǰ�ı�߲�ͬ����ʹ������Ϊƽ����б�����Ƿ����?";
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
        // strValue ����7/1/0�ǲ��Ϸ��ģ���Ҫ�����ж�
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
                strErrMsg = QString::fromLocal8Bit("����������");
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
            if (dTemp - c_dLength > 0.5)//�빰
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
//     { //�����м����α�ߣ�������������Ҫ����������
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
//     //�����м����α��
//     IGMPElementDrawObj *pEdoCurr = pPrev;
//     while (pEdoCurr != nullptr)
//     {
//         IGMPCustomLineSolidShape *pShapeNext = dynamic_cast<IGMPCustomLineSolidShape*>(pEdoCurr->shape());
//         if (pShapeNext != nullptr)
//         {
//             //�ж��Ƿ�ƽ��
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
//         //�жϱ���Ƿ����
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
//     //�����м����α��
//     IGMPElementDrawObj *pNext = pPrev;
//     while (pNext != nullptr)
//     {
//         IGMPCustomLineSolidShape *pShapeNext = dynamic_cast<IGMPCustomLineSolidShape*>(pNext->shape());
//         if (pShapeNext == nullptr)
//             continue;
// 
//         //�жϱ���Ƿ����
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
//     //������պ�ָ�Ĭ��ֵ
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
//     //��������߱仯
//     IGMPElevOperator *const c_pElevOper = pPrev->contnr()->model()->oprCenter()->elevOpr();
//     double const c_dEndOldValue = pLast->properties()->asInteger(pfnEndPtTopElev);
//     double const c_dStartOldValue = pPrev->properties()->asInteger(pfnStartPtTopElev);
//     double c_dStartNewValue = c_pElevOper->calcElev(pPrev, c_StartPrev, strNewValue);//c_pElevOper->calcElev(pPrev, c_StartPrev, strOldValue);
//     double c_dEndNewValue = c_pElevOper->calcElev(pPrev, c_EndLast, strNewValue);
//     //
//     if (pProp->propName() == pfnStartPtTopElev)
//     {
//         //���������
//         GMPPropInfoDefaultWriter::writeProp(pPrev->properties()->propByName(pProp->propName()), strNewValue, strOldValue);
//         calcNormalBeamElve(pProp, pPrev, dLength, dLengthNext, c_dEndOldValue, c_dStartNewValue);
//     }
//     else
//     {
//         //�����յ���
//         GMPPropInfoDefaultWriter::writeProp(pLast->properties()->propByName(pProp->propName()), strNewValue, strOldValue);
//         calcSpecialBeamElve(pProp, pPrev, dLength, dLengthNext, c_dStartOldValue, c_dEndNewValue);
//     }
// }
// 
// void GTJArchBeamElevPropInfoWriter::calcNormalBeamElve(GMPPropPtr pProp, IGMPElementDrawObj *pEdo,
//     double dLength, double dLengthNext, double dOldElve, double dNewElve)
// {
//     auto safeSetProp = [&] (GMPPropPtr prop, const GString&sValue) {
//         // ��λ��
//         GString strTransfer;
//         if (!m_pUnitTransfer || !m_pUnitTransfer->display2Store(prop, sValue, strTransfer, false))
//         {
//             strTransfer = sValue.trimmed();
//         }
//         // ����������������ֵ�����Ŀǰֻ����gmdtText����Ϊ���ݿ��ж�Ӧ����Memo
//         prop->setAsString(strTransfer);
//     };
//     //���������
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
//     //���������
//     safeSetProp(pEdo->properties()->propByName(pfnEndPtTopElev), QString::number(dTempElve / 1000));
//     //ѭ��������Щ����
//     IGMPRelationOperator *pRelaOpr = pEdo->contnr()->model()->oprCenter()->relaOpr();
//     IGMPElementDrawObj *pNext = pRelaOpr->next(pEdo);
//     while (pNext != nullptr)
//     {
//         //�������
//         safeSetProp(pNext->properties()->propByName(pProp->propName()), QString::number(dTempElve / 1000));
// 
//         //�����¸������
//         IGMPCustomLineSolidShape *pShapeNext = dynamic_cast<IGMPCustomLineSolidShape*>(pNext->shape());
//         if (pShapeNext == nullptr)
//         {
//             pNext = pRelaOpr->next(pNext);
//             continue;
//         }
//         //���ݲ�ͬ������㲻ͬ���
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
//         //�յ�����
//         safeSetProp(pNext->properties()->propByName(pfnEndPtTopElev), QString::number(dTempElve / 1000));
//         pNext = pRelaOpr->next(pNext);
//     }
// }
// 
// void GTJArchBeamElevPropInfoWriter::calcSpecialBeamElve(GMPPropPtr pProp, IGMPElementDrawObj *pEdo, double dLength,
//     double dLengthNext, double dOldElve, double dNewElve)
// {
//     auto safeSetProp = [&] (GMPPropPtr prop, const GString&sValue) {
//         // ��λ��
//         GString strTransfer;
//         if (!m_pUnitTransfer || !m_pUnitTransfer->display2Store(prop, sValue, strTransfer, false))
//         {
//             strTransfer = sValue.trimmed();
//         }
//         // ����������������ֵ�����Ŀǰֻ����gmdtText����Ϊ���ݿ��ж�Ӧ����Memo
//         prop->setAsString(strTransfer);
//         //prop->setAsDouble(strTransfer.toDouble() * 1000);
//     };
// 
//     //���������
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
//     //���������
//     safeSetProp(pEdo->properties()->propByName(pfnEndPtTopElev), QString::number(dTempElve / 1000));
//     //ѭ��������Щ����
//     IGMPRelationOperator *pRelaOpr = pEdo->contnr()->model()->oprCenter()->relaOpr();
//     IGMPElementDrawObj *pNext = pRelaOpr->next(pEdo);
//     while (pNext != nullptr)
//     {
//         //�������
//         safeSetProp(pNext->properties()->propByName(pfnStartPtTopElev), QString::number(dTempElve / 1000));
// 
//         //�����¸������
//         IGMPCustomLineSolidShape *pShapeNext = dynamic_cast<IGMPCustomLineSolidShape*>(pNext->shape());
//         if (pShapeNext == nullptr)
//         {
//             pNext = pRelaOpr->next(pNext);
//             continue;
//         }
//         //���ݲ�ͬ������㲻ͬ���
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
//         //�յ�����
//         //��������һ��Ͳ��޸��ˣ�ӦΪ֮ǰ���Ѿ��޸Ĺ���
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
//         // ��λ��
//         GString strTransfer;
//         if (!m_pUnitTransfer || !m_pUnitTransfer->display2Store(prop, sValue, strTransfer, false))
//         {
//             strTransfer = sValue.trimmed();
//         }
//         // ����������������ֵ�����Ŀǰֻ����gmdtText����Ϊ���ݿ��ж�Ӧ����Memo
//         prop->setAsString(strTransfer);
//     };
// 
//     //����Ĭ��ֵ�Ϳ�ֵ�Ĵ���
//     // �ָ�Ĭ��ֵ
//     //�������˼�����Ϊ�߶ȵĵ�λ��mm����ߵĵ�λ��m
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
//     //2�����̨����ֻ��ʾ�ױ��
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
//                 //��ά����άƽ�е�ֱ����
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
//                 //��άƽ�еķ����ߵ�������
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
//         // ��λ��
//         GString strTransfer;
//         if (!m_pUnitTransfer || !m_pUnitTransfer->display2Store(prop, sValue, strTransfer, false))
//         {
//             strTransfer = sValue.trimmed();
//         }
//         // ����������������ֵ�����Ŀǰֻ����gmdtText����Ϊ���ݿ��ж�Ӧ����Memo
//         prop->setAsString(strTransfer);
//     };
//     //����Ĭ��ֵ�Ϳ�ֵ�Ĵ���
//     // �ָ�Ĭ��ֵ
//     //�������˼�����Ϊ�߶ȵĵ�λ��mm����ߵĵ�λ��m
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
//     //��������߱仯
//     IGMPElevOperator *const c_pElevOper = pPrev->contnr()->model()->oprCenter()->elevOpr();
//     //��̨��
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
//         //���������
//         GMPPropInfoDefaultWriter::writeProp(pPrev->properties()->propByName(pProp->propName()), strNewValue, strOldValue);
//         safeSetProp(pPrev->properties()->propByName(pfnStartPtBottomElev), strNewValue + "-" + QString::number(dStartSectionHeight));
//         calcNormalFDBeamElve(pProp, pPrev, dLength, dLengthNext, c_dEndOldValue, c_dStartNewValue - dStartSectionHeight * 1000);
//     }
//     else if (pProp->propName() == pfnEndPtTopElev)
//     {
//         //�����յ���
//         GMPPropInfoDefaultWriter::writeProp(pLast->properties()->propByName(pProp->propName()), strNewValue, strOldValue);
//         safeSetProp(pLast->properties()->propByName(pfnEndPtBottomElev), strNewValue + "-" + QString::number(dEndSectionHeight));
//         calcSpecialFDBeamElve(pProp, pPrev, dLength, dLengthNext, c_dStartOldValue, c_dEndNewValue - dEndSectionHeight * 1000);
//     }
//     else if (pProp->propName() == pfnStartPtBottomElev)
//     {
//         //���������
//         GMPPropInfoDefaultWriter::writeProp(pPrev->properties()->propByName(pProp->propName()), strNewValue, strOldValue);
//         safeSetProp(pPrev->properties()->propByName(pfnStartPtTopElev), strNewValue + "+" + QString::number(dStartSectionHeight));
//         calcNormalFDBeamElve(pProp, pPrev, dLength, dLengthNext, c_dEndOldValue, c_dStartNewValue);
//     }
//     else
//     {
//         //���������
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
//         // ��λ��
//         GString strTransfer;
//         if (!m_pUnitTransfer || !m_pUnitTransfer->display2Store(prop, sValue, strTransfer, false))
//         {
//             strTransfer = sValue.trimmed();
//         }
//         // ����������������ֵ�����Ŀǰֻ����gmdtText����Ϊ���ݿ��ж�Ӧ����Memo
//         prop->setAsString(strTransfer);
//     };
//     double dSectionHeight = pProp->owner()->asInteger(pfnSectionHeight) / 1000.0;
//     //���������
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
//     //���������
// 
//     safeSetProp(pEdo->properties()->propByName(pfnEndPtBottomElev), QString::number(dTempElve / 1000));
//     safeSetProp(pEdo->properties()->propByName(pfnEndPtTopElev), QString::number(dTempElve / 1000 + dSectionHeight));
//     //ѭ��������Щ����
//     IGMPRelationOperator *pRelaOpr = pEdo->contnr()->model()->oprCenter()->relaOpr();
//     IGMPElementDrawObj *pNext = pRelaOpr->next(pEdo);
//     while (pNext != nullptr)
//     {
//         //�������
//         dSectionHeight = pNext->properties()->asInteger(pfnSectionHeight) / 1000.0;
//         safeSetProp(pNext->properties()->propByName(pfnStartPtBottomElev), QString::number(dTempElve / 1000));
//         safeSetProp(pNext->properties()->propByName(pfnStartPtTopElev), QString::number(dTempElve / 1000 + dSectionHeight));
// 
//         //�����¸������
//         IGMPCustomLineSolidShape *pShapeNext = dynamic_cast<IGMPCustomLineSolidShape*>(pNext->shape());
//         if (pShapeNext == nullptr)
//         {
//             pNext = pRelaOpr->next(pNext);
//             continue;
//         }
//         //���ݲ�ͬ������㲻ͬ���
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
//         //�յ�����
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
//         // ��λ��
//         GString strTransfer;
//         if (!m_pUnitTransfer || !m_pUnitTransfer->display2Store(prop, sValue, strTransfer, false))
//         {
//             strTransfer = sValue.trimmed();
//         }
//         // ����������������ֵ�����Ŀǰֻ����gmdtText����Ϊ���ݿ��ж�Ӧ����Memo
//         prop->setAsString(strTransfer);
//         //prop->setAsDouble(strTransfer.toDouble() * 1000);
//     };
//     double dSectionHeight = pProp->owner()->asInteger(pfnSectionHeight) / 1000.0;
//     //���������
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
//     //���������
//     safeSetProp(pEdo->properties()->propByName(pfnEndPtBottomElev), QString::number(dTempElve / 1000));
//     safeSetProp(pEdo->properties()->propByName(pfnEndPtTopElev), QString::number(dTempElve / 1000 + dSectionHeight));
//     //ѭ��������Щ����
//     IGMPRelationOperator *pRelaOpr = pEdo->contnr()->model()->oprCenter()->relaOpr();
//     IGMPElementDrawObj *pNext = pRelaOpr->next(pEdo);
//     while (pNext != nullptr)
//     {
//         //�������
//         dSectionHeight = pNext->properties()->asInteger(pfnSectionHeight) / 1000.0;
//         safeSetProp(pNext->properties()->propByName(pfnStartPtBottomElev), QString::number(dTempElve / 1000));
//         safeSetProp(pNext->properties()->propByName(pfnStartPtTopElev), QString::number(dTempElve / 1000 + dSectionHeight));
// 
//         //�����¸������
//         IGMPCustomLineSolidShape *pShapeNext = dynamic_cast<IGMPCustomLineSolidShape*>(pNext->shape());
//         if (pShapeNext == nullptr)
//         {
//             pNext = pRelaOpr->next(pNext);
//             continue;
//         }
//         //���ݲ�ͬ������㲻ͬ���
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
//         //�յ�����
//         //��������һ��Ͳ��޸��ˣ�ӦΪ֮ǰ���Ѿ��޸Ĺ���
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
// �ж�@pEdo�Ƿ�Ϊ���������
// @strStartPt �� @strEndPt �ֱ�Ϊ���������յ����������
// @lengVec���������������ĳ��ȣ���ֱ�����������˵�ľ���Ϊ׼
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
// ���ö�������ı��
// @strStartPt �� @strEndPt �ֱ�Ϊ���������յ����������
// @dHeight Ϊ���������������յ�ı�߲�
void GTJFDBeamElevPropInfoWriter::configSegmentsElev(IGMPRelationOperator *const pRelaOpr,
    IGMPElementDrawObj *const pEdo, const std::wstring &strStartPt, const std::wstring &strEndPt, const double dHeight,
    const std::vector<double> &lengthVec)
{
    IGMPElementDrawObj *pEdoCurr = pRelaOpr->first(pEdo);
    const GString strBaseElev = pEdoCurr->properties()->asString(strStartPt);
    int nCount = std::accumulate(lengthVec.cbegin(), lengthVec.cend(), 0);
    //modified by yangwf-a:nCount ����Ϊ0���Ӷ����³���ķ��գ�
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
// ��ԭֵΪ@strOldValue������@pProp����Ϊ@strNewValue
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
    { // �����̨��
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
    //Ϊɶ������Ȧ���Ľ���߶��õ��ַ�������һ������
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
                strErrMsg = strErrMsg.fromLocal8Bit("��������ߴ����,�������ɺϷ�ͼԪ,����������");
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
        //��������ĸ����ʾ
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
    //ȫ��ƽ����ֱ��д��
    if (m_dIsAllPlain)
    {
        return GMPPropInfoDefaultWriter::writeProp(pProp, newStr, strOldValue);
    }
    IGMPEObject *pObject = pProp->owner()->owner();
    //���ڹ������Ǳ����
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
    //�ж��Ƿ��Ǳ����
    GStringList strNewValue = newStr.split('/');
    if (strNewValue.size() > 1)
    {
        //�� 500/500 ��ʽ����Ϊ�Ǳ����
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
                "��ǰ�����´��ڹ���/���ι���ͼԪ,�޸�Ϊ��������󽫻�ʧȥ����Ϣ,�Ƿ����?";
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

//б����ı�߱�ƽ�������ﴦ���޸ı�ߺ���Ϣ�ı仯
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
    //����ǹ�������Ĭ�ϵ�
    if (IGMPEObject::eoENT == pObject->type())
    {
        GMPPropInfoDefaultWriter::writeProp(pProp, oStrValue, strOldValue);
    }
    //�����ͼԪ����������¹���Ϣ
    else
    {
        GMPPropInfoDefaultWriter::writeProp(pProp, oStrValue, strOldValue);
        IGMPElementDrawObj* pParent = dynamic_cast<IGMPElementDrawObj*>(pObject);
        IGMPFaceSolidShape* pParentShape = dynamic_cast<IGMPFaceSolidShape*>(pParent->shape());
        //����ı��ֽ����Ƿ���ֱ���أ�����Ҫͬ�������poly��ʵ���������ֽ�������
        if (pProp->propName() == GTJDBCConsts::pfnEndConstruct)
        {
            //��������ֻ��Ҫ�����ʽ���Ƶ�����
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
    if (c_wsName == pfnSectionWidth)//�����������������������
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
    if (c_wsName == pfnAxisOffset)//�������뱣��
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
    //������ˮ�ӣ�ʹ�価����������
    double dNewValue = strNewValue.toDouble();
    double dOldValue = strOldValue.toDouble();
    if (ggp::compareValue(dNewValue, dOldValue, g_DistEpsilon) == vrGreaterThan)
    {
        AdjustSumpCoordinate(pProp, dNewValue, dOldValue);
    }
    //д������
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
    //TRUE ���ÿ�  FALSE ���ó�
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

    //ȡ��ˮ�Ӻ͸��غϵı�
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

    //ͨ��ƫ�Ƽ�ˮ�Ӿֲ�����ϵ���ﵽ����������Ч��
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
 * @brief    �������Ա༭����
 * @author
 * @date     2015��12��02��
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
        //�ӻָ�Ĭ��ֵ
        pProp->initDefaultValue();

        changeEDOAxisOffset(pObj, pParentObj, dAxisOffset, false);

        //Ĭ��ֵ1000Ϊ������ԪĬ�Ͽ��
        if (ggp::IsGreaterThan(dAxisOffset, dTemp, getDefaultDistEpsilon()))
        {
            pParentObj->properties()->setIsNull(pfnAxisOffset);
            //��������
            changeEDOAxisOffset(pObj, pParentObj, dAxisOffset, true);
        }
        return;
    }

    // ��ֵΪ��or��ֵ���ʽ������,ps ����ʱע�͵������漰�κ�����
    //     GString strValue = strNewValue.trimmed();
    //     if (isNullValue(pProp, strValue))
    //     {
    //         pProp->setIsNull();
    // 
    //         changeEDOAxisOffset(pObj, pParentObj);
    //         return;
    //     }

    // ������ֵ���е�λ��
    GString strTransfer = GTJPropCommon::unitTransferElev(pProp, strNewValue, m_pUnitTransfer);
    //д�����Կ��
    pProp->setAsInteger(strTransfer.toInt());
    pObj->floor()->contnr()->model()->calculate();

    //���ͼԪ������߾���
    changeEDOAxisOffset(pObj, pParentObj, dAxisOffset, false);
    double dNewAxisOffset = strTransfer.toDouble() / 2.0;
    if (ggp::IsGreaterThan(dAxisOffset, dNewAxisOffset, getDefaultDistEpsilon()))
    {
        //�޸Ĺ�������
        pParentObj->properties()->setIsNull(pfnAxisOffset);
        //����ͼԪ����
        changeEDOAxisOffset(pObj, pParentObj, dAxisOffset, true);
    }

    if (pObj->type() == IGMPEObject::eoEDO)
    {
        //��Ԫ��������AxisOffset�����Ĭ��AxisOffset����󣬴�ʱ���������ã�
        //��Ҫ�Ե�Ԫ��������AxisOffset����ֵ��գ�Ȼ�����Ĭ��ֵ����
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
 * @brief    ��ȡͼԪ����߾���
 * @author
 * @date     2015��12��02��
 */
void GTJStripFDUnitPropInfoWriter::changeEDOAxisOffset(IGMPEObject *pObj1, IGMPEObject *pObj2, double &dAxisOffset, bool isNull)
{
    //�޸Ĺ���������ͼԪ����
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
            //modified by zhangh-t ֻ�����޸Ĺ�ƫ����ʱ�򣬲��б�Ҫ�����������
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
            { // ���ý����ȵ�˽������ֵ
                pPropList->setAsInteger(pfnSectionWidthExt, nValue);
            }
            // ������������С�����߾�����߾���ʱ�������߾�����߾���ָ�ΪĬ��ֵ
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
//ֱ���ݶ� 
GTJFlightPropWriter::GTJFlightPropWriter(vector<int> *const pAcceptEntTypes) : GMPPropInfoDefaultWriter(pAcceptEntTypes)
{
    m_AcceptProps.clear();
    m_AcceptProps.push_back(GString::fromStdWString(GTJDBCConsts::pfnFlightHeight));
    m_AcceptProps.push_back(QString::fromStdWString(GTJDBCConsts::pfnStepHeight));
}

void GTJFlightPropWriter::writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)
{
    const std::wstring &c_wsName = pProp->propName();
    if (GTJDBCConsts::pfnFlightHeight == c_wsName || GTJDBCConsts::pfnStepHeight == c_wsName)//�������뱣��
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
* ����: validateProp, add for fix GTJ-10294
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

//�����ݶ� 
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
* ����: validateProp, add for fix GTJ-10294
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

    //����ģʽ�µĶ���
    Q_ASSERT(pProp);
    Q_ASSERT(pProp->owner());
    Q_ASSERT(pProp->owner()->hasProp(pfnStartPtBottomElev));

    //�ú�����prop��ʾ�����Ե�ֵ��ΪsValue
    auto safeSetProp = [&] (GMPPropPtr prop, const GString&sValue) {
        // ��λ��
        GString strTransfer = GTJPropCommon::unitTransferElev(prop, sValue, m_pUnitTransfer);
        // ����������������ֵ�����Ŀǰֻ����gmdtText����Ϊ���ݿ��ж�Ӧ����Memo
        prop->setAsString(strTransfer);
    };

    //����Ĭ��ֵ�Ϳ�ֵ�Ĵ����ѵع��ĵױ�ߣ����ױ�ߣ��յ�ױ����Ϊ��ױ��
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

    //����ױ�߲�Ϊ�գ������ױ�ߺ��յ�ױ�ߵ�ֵ��Ϊ�ױ��
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
* @brief:		�������ԣ��@ȡ�����Ę���
* @param[in]	pProp --
* @return		����ָ�
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
* @brief:		һ�M���Խӿڣ�������section info��Ϣ�Ƿ�͵�һ�����Խӿڵ�һ��
* @param        vector<GMPPropPtr>
* @return		��һ�����Խӿڵ�section info�Єe춵�һ�����ԣ��t�� true
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
* @brief:		�������ԣ��@ȡ������ԭʼ����
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

        // �x������DԪ��ԭʼ��������Ҫ���}ӛ�
        if (vOrgElements.end() == vOrgElements.find(pOrgElement))
        {
            vOrgElements.insert(pOrgElement);
        }
    }
}

void GTJTrenchPropInfoWriter::onButtonClick(vector<GMPPropPtr>& oProps, bool& bModify)
{
    // ����ǆ΂��DԪ���x�У��tֻ���ڽ���������޸ĵ���r�£�����Ҫ�������O��
    // ����Ƕ����DԪ���x�У����Д��¸������Ľ�������Ƿ�һ�£�������Բ�һ��
    // ʱ�������O�õ�����_�J����������Ҫ�������O��
    bool bForceUpdate = isSecInfoPropDiff(oProps);

    IGMPProperties* pProps = oProps[0]->owner();
    IGMPEObject* pObj = pProps->owner();

    // �����Π��ǹ��Ќ��ԣ��@�e�������԰ь����Ę���ָ�ӛ��������֮���O
    // �Ì��ԡ�ԭ���ж���Ҋ����������ע�
    set<IGMPElement *>vOrgElements;
    getOrgElement(oProps, vOrgElements);

    // ���������x�У��������Π��{���r�����յ�һ���DԪ���������Ľ����Π�����O�ó�ʼֵ
    // �ҵ���һ������
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

    // new ����� Widget ָᘣ� ���� SelParamPolyForm ���߁Kؓ؟ጷţ��ʴ�̎�� new ����Ҫ delete ����
    GTJCustomParamWidgetDefault *pCustomWidget = new GTJCustomParamWidgetDefault();

    QString sProps = pProps->asString(L"SectionInfo");
    GTJSelParamPolyForm selParamPoly(m_pService->model()->paramPolyDB(), pObj, sProps, -1, pCustomWidget, QApplication::activeWindow());

    if (selParamPoly.exec() != QDialog::Accepted) return;

    //
    bModify = (0 != sProps.compare(selParamPoly.selParamValue()));

    // �������޸���o׃�ӣ��t�ŗ����m���޸�
    // ���O��֮ǰ�����в�e���tֻҪ���_�J�����ͱ���O�Ì���
    if (bModify || bForceUpdate)
    {
        sProps = selParamPoly.selParamValue();

        GTJParamTrenchSectionInfoParser teSectionParser;

        // �����Π���Ա��޸��ˣ��K�ң����е؜φ�Ԫ��׃�ɟoЧ����
        if (0 == teSectionParser.validUnitCounts(sProps))
        {
            // �z�y��Ԫ��Ч�ԣ���ʾ	
            if (GMPMessageBox::question(QApplication::activeWindow(),
                qApp->translate(c_szPropInfoWriter, c_Confrim), qApp->translate(c_pGTJPropInfoWriter, c_strInvalidParamTrenchUnits),
                GlodonMessageBox::No | GlodonMessageBox::Yes, GlodonMessageBox::Yes) == GlodonMessageBox::No)
            {
                return;
            }
        }

        // ֮����ʹ�Ø����������O�ã�ԭ���ж���
        // 1. �����Π��ǹ����Ќ��ԣ���ʹ��ǰ�Ñ��x����ǈDԪ�����������޸ģ�ʹ�Ì����Ę����Č�����ȥ�O��Ҳ�ǿ��Ե�
        // 2. ����ԭ���x�е؜ψDԪ�����{���˘����Ľ����Π�֮�ᣬ�؜φ�Ԫ���ܟoЧ���oЧ�ĵ؜φ�Ԫ�����h���������ĈDԪҲ�����h������ζ���x�еĈD
        //    ԪҲ�����h������������m�������F�ڱ��h���ğoЧ�DԪ���������޸ģ��@�eʹ�ÈDԪ���������Č���ȥ��
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

//dongxd-a ���������޸�ʱͬ����ʵ��ȣ����ΪĬ�Ͽ�ʱ��ͬ��
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
            //modified by zhangh-t �޸Ĺ��Ĺ����������ظ��޸�
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
                        strErrMsg = strErrMsg.fromLocal8Bit("������ (0,100000] ֮��������� �� ���߾�����߾���");
                        return false;
                    }
                }
                else
                {
                    const double dTemp = pProp->owner()->asDouble(pfnAxisOffset);
                    if (pSectionHeight - dTemp < -0.00001)
                    {
                        strErrMsg = strErrMsg.fromLocal8Bit("������ (0,100000] ֮��������� �� ���߾�����߾���");
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
    //��ǽ�����ǽ�����д���ֱ��д��
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

    //�ǲ�����¥�ݲ�����
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
                //���ݸ��ĺ�Ĺ��������޸��ӹ����еĹ������Ʋ�����
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
    m_AcceptProps.push_back(GString::fromStdWString(L"WallOutExpandElev"));  //̤���߶�
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
        // �˜ʈD�����Q�����O��
        pProps->setAsString(GTJDBCConsts::pfnStdDrawingsName, oFrm.getDrawingSetName());
        // �@ȡ�˜ʈD���a�����O��
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
//��
bool GTJDrawingSetDoorPropInfoWriter::saveProperties(IGMPProperties* pProps, GSPRecord pRecord)
{
    if (nullptr == pProps || nullptr == pRecord) return false;
    
    // ���ڌ���    OpeningWidth
    pProps->setAsInteger(pfnOpeningWidth, pRecord.asInteger(GString::fromWCharArray(pfnOpeningWidth)), false);

    // ���ڸ߶�    OpeningHeight
    pProps->setAsInteger(pfnOpeningHeight, pRecord.asInteger(GString::fromWCharArray(pfnOpeningHeight)), false);

    // ���        FrameThickness
    pProps->setAsInteger(pfnFrameThickness, pRecord.asInteger(GString::fromWCharArray(pfnFrameThickness)), false);

    // �����ҿ۳ߴ� FillingWidth
    pProps->setAsInteger(L"FillingWidth", pRecord.asInteger(GString::fromWCharArray(pfnFrameLeftRightDeductSize)), false);

    // �����¿۳ߴ� FillingHeight
    pProps->setAsInteger(L"FillingHeight", pRecord.asInteger(GString::fromWCharArray(pfnFrameTopBottomDeductSize)), false);

    // ����Χ���  FrameArea
    pProps->setAsDouble(pfnFrameArea, pRecord.asFloat(GString::fromWCharArray(pfnFrameArea)) / 1000000, false);
    
    // �������    OpeningArea
    pProps->setAsInteger(pfnOpeningArea, pRecord.asInteger(GString::fromWCharArray(pfnOpeningArea)), false);

    return true;
}

// ��
bool GTJDrawingSetWindowPropInfoWriter::saveProperties(IGMPProperties* pProps, GSPRecord pRecord)
{
    if (nullptr == pProps || nullptr == pRecord) return false;

    // ���ڌ���    OpeningWidth
    pProps->setAsInteger(pfnOpeningWidth, pRecord.asInteger(GString::fromWCharArray(pfnOpeningWidth)), false);

    // ���ڸ߶�    OpeningHeight
    pProps->setAsInteger(pfnOpeningHeight, pRecord.asInteger(GString::fromWCharArray(pfnOpeningHeight)), false);

    // �x�ظ߶�		AboveFloorHeight
    pProps->setAsInteger(pfnAboveFloorHeight, pRecord.asInteger(GString::fromWCharArray(pfnAboveFloorHeight)), false);

    // ���        FrameThickness
    pProps->setAsInteger(pfnFrameThickness, pRecord.asInteger(GString::fromWCharArray(pfnFrameThickness)), false);

    // �����ҿ۳ߴ� FillingWidth
    pProps->setAsInteger(L"FillingWidth", pRecord.asInteger(GString::fromWCharArray(pfnFrameLeftRightDeductSize)), false);

    // �����¿۳ߴ� FillingHeight
    pProps->setAsInteger(L"FillingHeight", pRecord.asInteger(GString::fromWCharArray(pfnFrameTopBottomDeductSize)), false);

    // ����Χ���  FrameArea
    pProps->setAsDouble(pfnFrameArea, pRecord.asFloat(GString::fromWCharArray(pfnFrameArea)) / 1000000, false);

    // �������    OpeningArea
    pProps->setAsInteger(pfnOpeningArea, pRecord.asInteger(GString::fromWCharArray(pfnOpeningArea)), false);

    return true;
}

// ������
bool GTJDrawingSetDoorWinPropInfoWriter::saveProperties(IGMPProperties* pProps, GSPRecord pRecord)
{
    if (nullptr == pProps || nullptr == pRecord) return false;

    // ���ڌ���    OpeningWidth
    pProps->setAsInteger(pfnOpeningWidth, pRecord.asInteger(GString::fromWCharArray(pfnOpeningWidth)), false);

    // ���ڸ߶�    OpeningHeight
    pProps->setAsInteger(pfnOpeningHeight, pRecord.asInteger(GString::fromWCharArray(pfnOpeningHeight)), false);

    // ���        FrameThickness
    pProps->setAsInteger(pfnFrameThickness, pRecord.asInteger(GString::fromWCharArray(pfnFrameThickness)), false);

    // ������
    pProps->setAsInteger(GTJDBCConsts::pfnWinWidth, pRecord.asInteger(GString::fromWCharArray(pfnDWWinWidth)), false);

    // ���x�ظ߶�
    //pProps->setAsInteger(GTJDBCConsts::pfnWinAboveFloorHeight, pRecord.asInteger(GString::fromWCharArray(GTJDBCConsts::pfnWinAboveFloorHeight)), false);
    
    // �T�����ҿ۳ߴ�
    pProps->setAsInteger(L"DoorFillingWidth", pRecord.asInteger(GString::fromWCharArray(pfnDoorFrameLeftRightDeductSize)), false);

    // �T�����¿۳ߴ�
    pProps->setAsInteger(L"DoorFillingHeight", pRecord.asInteger(GString::fromWCharArray(pfnDoorFrameTopBottomDeductSize)), false);

    // �������ҿ۳ߴ�  WinFillingWidth //WinFrameLeftRightDeductSize
    pProps->setAsInteger(L"WinFillingWidth", pRecord.asInteger(GString::fromWCharArray(pfnWinFrameLeftRightDeductSize)), false);

    // �������¿۳ߴ�	  WinFillingHeight
    pProps->setAsInteger(L"WinFillingHeight", pRecord.asInteger(GString::fromWCharArray(pfnWinFrameTopBottomDeductSize)), false);

    // �������    OpeningArea
    pProps->setAsInteger(pfnOpeningArea, pRecord.asInteger(GString::fromWCharArray(pfnOpeningArea)), false);

    // ����Χ���  FrameArea
    pProps->setAsDouble(L"WinFrameArea", pRecord.asFloat(GString::fromWCharArray(pfnFrameArea)) / 1000000, false);
    
    return true;
}

//����
bool GTJDrawingSetLintelPropInfoWriter::saveProperties(IGMPProperties* pProps, GSPRecord pRecord)
{
    if (nullptr == pProps || nullptr == pRecord) return false;

    // ����������    ConcreteUsage
    //pProps->setAsInteger(pfnOpeningWidth, pRecord.asInteger(GString::fromWCharArray(pfnOpeningWidth)));

    // 䓽��    ReinfUsage
    //pProps->setAsInteger(pfnOpeningHeight, pRecord.asInteger(GString::fromWCharArray(pfnOpeningHeight)));

    // �L��       Len ok
    pProps->setAsInteger(pfnLen, pRecord.asInteger(GString::fromWCharArray(pfnLen)), false);

    // ���挒��	 SectionWidth ok
    pProps->setAsInteger(pfnSectionWidth, pRecord.asInteger(GString::fromWCharArray(pfnSectionWidth)), false);

    // ����߶�	 SectionHeight ok
    pProps->setAsInteger(pfnSectionHeight, pRecord.asInteger(GString::fromWCharArray(pfnSectionHeight)), false);

    // ͹������߶�	LSectionHeight
    //pProps->setAsInteger(L"FillingHeight", pRecord.asInteger(GString::fromWCharArray(pfnFrameTopBottomDeductSize)));

    // ͹�����挒��	LSectionWidth
    //pProps->setAsInteger(L"FillingHeight", pRecord.asInteger(GString::fromWCharArray(pfnFrameTopBottomDeductSize)));

    // �w�e		Volume  ok
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
