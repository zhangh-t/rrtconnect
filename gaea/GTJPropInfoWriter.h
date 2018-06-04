/*!
*@file      GGJPropInfoWriter.h
*@brief     ��Ʒ�������Դ���
*@author    songzm
*@date      2014/09/15
*@remarks
*@version   3.0
*@Copyright (c) 1998-2013 Glodon Corporation
*/
#ifndef GTJPROPINFOWRITER_H
#define GTJPROPINFOWRITER_H

#include "GMModel/GMPException.h"
#include "GMPCore/GMPServiceIntf.h"
#include "GMPCore/GMPParamNAbnormalWriter.h"
#include "GMPCore/GMPPropInfoWriter.h"
#include "GMModel/IGMPNotifyOperator.h"
#include "GMModel/IGMPPropertySchema.h"

#include "GTJDBConsts.h"
#include "GTJCommon/GTJInfoDlg.h"
#include "GTJCommon_global.h"
#include "GSPCore.h"
#include "Algorithm/algpositionJudge.h"

class GTJCOMMON_EXPORT GTJParamNAbnormalWriter : public GMPParamNAbnormalWriter
{
public:
    GTJParamNAbnormalWriter(ggp::CDatabase* pParamDB, vector<int> * pAcceptEntTypes = nullptr);
    virtual ~GTJParamNAbnormalWriter() { }
public:
    virtual void onWriteProp(IGMPProperties* pProperties, const GMPParamSectionInfo * pSectionInfo);
public:
    virtual void onButtonClick(vector<GMPPropPtr>& oProps, bool& bModify);
    virtual void onWriteAbSectionProp(IGMPProperties* pProperties, const GMPParamSectionInfo * pSectionInfo) { }
protected:
    virtual void editAbnormitySection(int nElementTypeID, vector<GMPPropPtr>& oProps, bool& bModify);
    virtual void setPoly(IGMPProperties * pProperties, CPolygon * pPoly);
    void clearEdgeInfos(IGMPEObject* pObj);
    void updateAxisOffset(IGMPProperties *const pPropVec, const double dWidth);
    bool getProperWidthProperty(IGMPProperties *const pPropVec, double &dWidth);
    IGMPEObject* getParentObject(IGMPEObject *const pChild);
    void changeEdoAxisOffset(IGMPProperties* pPropVec, IGMPEObject * pObj, IGMPEObject* pChildObj, const double dWidth = -1);

protected:
    ggp::CDatabase* m_pParamDB;
    //added by zhangh-t ���׮��̨��Ԫ�ظ�������Ч������
    virtual void afterModifySectionInfo(vector<GMPPropPtr>& oProps);
    bool isModified(IGMPEObject* pObj);
    std::set<IGMPElement*> m_setModifiedElements;
};

/**
* @brief ר�Ŵ����ӹ����ı����������������׮��̨ ���������ȣ�
*ԭ�� �߶Ⱦ������ֲ���
*����޸Ķ���� ��ͬʱ�޸ĵͱ��
*����޸ĵױ�� ��ͬʱ�޸Ķ����
*����޸ĸ߶� ��ͬʱ�޸Ķ����
*
* @remark ע���� GGJUIPlugin::registerPropWriters
*
* @class GTJComplexPropWriter
*/
class GTJCOMMON_EXPORT GTJComplexPropWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJComplexPropWriter(vector<int> * pAcceptEntTypes = nullptr);
    ~GTJComplexPropWriter();

public:
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)override;
};

/**
* @brief ר�Ŵ����ӹ���(��׮��̨��Ԫ)�������ɫ���߿���ɫ����͸���ȵ���������
* ԭ��(1) ���������Ըı䣬��Ԫ�����游�����ı�(ǰ���ǵ�Ԫ��������Ϊ��ֵ)
*       (2) ��Ԫ�������Ըı䣬��Ӱ�츸��������
*       (3) �޸ĸ�ʱ�����ǿ�ֵʱ���ӱ���ͻ�����
*       (4) �޸���ʱ���ж�����͸�һ��  �ͱ�Ϊ��ֵ
*
* @class GTJComplexDisplayStylePropWriter
*/
class GTJCOMMON_EXPORT GTJComplexDisplayStylePropWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJComplexDisplayStylePropWriter(vector<int> * pAcceptEntTypes = nullptr);
    ~GTJComplexDisplayStylePropWriter();

public:
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)override;
};

/**
* @brief ר�Ŵ�����У������ �ױ�� �͸߶ȣ��Ĺ����ı����������������
* ԭ�� ��Ⱦ������ֲ���
* ���->��
* ��->��
* ��->��
* @class GTJRaftSlabElevPropWriter
*/
class GTJCOMMON_EXPORT GTJRaftSlabElevPropWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJRaftSlabElevPropWriter(vector<int> * pAcceptEntTypes = nullptr);
    ~GTJRaftSlabElevPropWriter();

public:
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)override;
    void safeSetProp(GMPPropPtr prop, const GString&sValue);
};

/*!
*@brief  ������ǽ���㰴ť����
*@author wangxb 2014��12��5��
*@return
*/
class GTJCOMMON_EXPORT GTJParamSwingWinPropWriter : public GMPParamNAbnormalWriter
{
public:
    GTJParamSwingWinPropWriter(std::vector<int> *pAcceptEntTypes, IGMPService* pService);
public:
    virtual void onButtonClick(std::vector<GMPPropPtr>& oProps, bool& bModify);

private:
    IGMPService* m_pService;
};

/**
* @brief ר�Ŵ����ӹ����ĸ������ĸ߶����ԣ�����׮��̨ ���������ȣ�
*
* @class GGJComplexHeightLiserner
*/
//ltPropListener | ltEntListener | ltEdoListener
class GTJCOMMON_EXPORT GTJComplexHeightListener : public IGMPRelaDataListener
{
public:
    GTJComplexHeightListener(IGMPService *pIGMPService);
    //��ʼ�����޸�֪ͨ
    virtual void onBeginEdit();
    //���������޸�֪ͨ
    virtual void onEndEdit();

    virtual void onNotify(int iNotifyType, IGMPProperties* pProps, void* pData, bool bRedoUndo);
    //����
    virtual void onNotify(int iNotifyType, IGMPElement* pEnt, void* pData, bool bRedoUndo);
    //ͼԪ
    virtual void onNotify(int iNotifyType, IGMPElementDrawObj* pEdo, void* pData, bool bRedoUndo);
private:
    std::map<IGMPProperties*, IGMPPropertySchema*> m_oMap;
    IGMPService     *m_pIGMPService;
};

////////////////////////////////////////������ñ�����ա����͡�����/////////////////////////////////////
class GTJCOMMON_EXPORT GTJSpecialParamTypeWriter : public GMPPropInfoWriter
{
public:
    GTJSpecialParamTypeWriter(ggp::CDatabase* pParamDB, vector<int> * pAcceptEntTypes = nullptr);

    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
    virtual bool validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg);
    virtual void onButtonClick(vector<GMPPropPtr>& oProps, bool& bModify);

private:
    ggp::CDatabase* m_pParamDB;
};

/////////////////////////////////////�󽽴��������ʹ���/////////////////////////////////////////////////
class GTJCOMMON_EXPORT GTJPostCastStripTypeWriter : public GMPPropInfoWriter
{
public:
    GTJPostCastStripTypeWriter(ggp::CDatabase* pParamDB, vector<int> *pAcceptEntTypes = nullptr);

    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
    virtual bool validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg);
    virtual void onButtonClick(vector<GMPPropPtr>& oProps, bool& bModify);
protected:
    ggp::CDatabase *m_pParamDB;
};

/*!
*@file
*@brief    ������������ԡ���λ���롰��ߡ�����
*@author   zhaojs
*@date     2016.2.25
*@remarks
*@version 3.0
*Copyright (c) 1998-2013 Glodon Corporation
*/
class GTJCOMMON_EXPORT GTJLinetelBeamWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJLinetelBeamWriter(vector<int> * pAcceptEntTypes = nullptr);
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
    virtual bool validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg);

    enum LinetelHoldType
    {
        LINETEL_HOLD_UP = 0, // �����Ϸ�
        LINETEL_HOLD_DOWN = 1 // �����·�
    };

private:
    // �����Ĺ����������޸ĵĴ���
    bool updateElevOfArch(GMPPropPtr const pProp, const GString &strNewValue, const GString &strOldValue);
    // �ж��޸���/ʼ������ǽ�ľ�����Ƿ񳬹�Բ�ܳ�
    bool isOverCircumference(CCurve2d* pLine, int nPtLenInWall = 250, int nNewValue = 250);
     //���������߾���ǽƤ����ĺϷ���У��
    bool isValidAxisOffset(IGMPElementDrawObj* pEdo, double& dThickness, double& dCenterAxisOffset, GString& strErrMsg);
    //��ȡ�����ĸ��ཻ��ǽ������������
    void getWallByLintelParent(vector<IGMPElementDrawObj*> &oWallVector, IGMPElementDrawObj *pEDO, IGMPElementDrawObj *pParentEdo);
};

/**
* @brief ����ˮ�ӷ��º�ͨ�������Կ��޸ı�����Ϣ��ͳһ�޸����бߵı�����Ϣ
* ԭ�� ���ó��߾��룬���µ׿�/���½Ƕ�
* @class GTJSumpSlopInfoWriter
*/
class GTJCOMMON_EXPORT GTJSumpSlopInfoWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJSumpSlopInfoWriter(vector<int> * pAcceptEntTypes = nullptr);
    ~GTJSumpSlopInfoWriter();

public:
    virtual void writeProp(GMPPropPtr pProp, const GString &strNewValue, const GString &strOldValue) override;
    virtual bool validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg);
private:

    /*!
    *@brief
    *@author        jiamj-a 12.02.2014
    *@param[out]    IGMPElementDrawObj * pSumpEDO
    *@param[in]     const wstring & strPropName
    *@return        void
    */
    void writeEdgeInfos(IGMPElementDrawObj *pSumpEDO, const wstring &strPropName);
};

/*!
*@file
*@brief    ������������
*@author   zoul-a
*@date     2015��1��19��
*@remarks
*@version 3.0
*Copyright (c) 1998-2013 Glodon Corporation
*/
class GTJCOMMON_EXPORT GGJColumnBasePropInfoWriter : public GMPPropInfoDefaultWriter
{
public:
    GGJColumnBasePropInfoWriter(vector<int> * pAcceptEntTypes = nullptr);
public:
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
};

/*!
*@file
*@brief    �ֽ��������������
*@author   qinyb
*@date     2015��1��21��
*Copyright (c) 1998-2015 Glodon Corporation
*/
////////////////////////////////////////////�����������ֵ��////////////////////////////////////////////
class GTJCOMMON_EXPORT GTJSectionWidthWriter : public GMPSectionWidthWriter
{
public:
    GTJSectionWidthWriter(vector<int> * pAcceptEntTypes = nullptr);
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
};

////////////////////////////////////////////�����������ֵ��////////////////////////////////////////////
class GTJCOMMON_EXPORT GTJSectionHeightWriter : public GMPSectionHeightWriter
{
public:
    GTJSectionHeightWriter(vector<int> * pAcceptEntTypes = nullptr);
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
};

class GTJCOMMON_EXPORT GTJHandrailSectionWriter : public GMPParamNAbnormalWriter
{
public:
    GTJHandrailSectionWriter(ggp::CDatabase* pParamDB, vector<int> * pAcceptEntTypes = nullptr);
public:
    //virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
    virtual void onButtonClick(vector<GMPPropPtr>& oProps, bool& bModify);
};

/*!
*@file
*@brief    ������ۻ�������
*@author   zhangh-t
*@date     2015��3��27��
*@remarks
*@version 1.0
*Copyright (c) 1998-2013 Glodon Corporation
*/
class GTJCOMMON_EXPORT GCLDitchEarthPropInforWriter : public GMPPropInfoDefaultWriter, public CUpdateRecordObserver
{
public:
    GCLDitchEarthPropInforWriter(vector<int> * pAcceptEntTypes = nullptr);
    ~GCLDitchEarthPropInforWriter();

public:
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue)override;
    void safeSetProp(GMPPropPtr prop, const GString&sValue);
    virtual bool validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg)
    {
        const std::wstring &sName = pProp->propName();
        GString strValidValue = strValue;
        if (sName == pfnLeftSlopeFactor || sName == pfnRightSlopeFactor)
        {
            strValidValue = handleValue(strValue);
        }

        else if (sName == pfnLeftWorkingFaceWidth || sName == pfnRightWorkingFaceWidth)
        {
            strValidValue = handleInteger(strValue);
        }
        else if (sName == pfnBottomWidth)
        {
            // BEGIN: ADD BY 2015-05-07
            // �޸�ԭ�򣺵��޸ĵĲ۵׿�Ϊ�գ���ԭ���Ĭ��ֵС�����߾�����߾���ʱ
            //           Ӧ�ò������޸ģ���ʾ�û���������
            if (strValue.isEmpty())
            {
                // ������ֵ���е�λ��
                GString strDefaultBottomWidth;

                IGMPEObject* pObject = pProp->owner()->owner();
                strDefaultBottomWidth = pProp->schema()->defaultExpr();
                // ��֤ת�����ֵ�Ƿ�Ϸ�
                try
                {
                    pProp->check(strDefaultBottomWidth);
                }
                catch (GMPModelException e)
                {
                    strErrMsg = e.message();
                    return false;
                }
            }
            // END
        }
        return GMPPropInfoDefaultWriter::validateProp(pProp, strValidValue, strErrMsg);
    };
private:
    /*!
    *@brief  �������ϵ�����ʽ
    *@author zhangh-t 2015��3��27��
    *@param[in]    const QString& strNewValue
    *@return       QString
    */
    QString handleValue(const QString& strNewValue);

    /*!
    *@brief  ����200.00��������
    *@author zhangh-t 2015��3��27��
    *@param[in]    const QString& strNewValue
    *@return       QString
    */
    QString handleInteger(const QString& strNewValue);

    /*!
    *@brief  �����ϵ��
    *@author zhangh-t 2015��3��27��
    *@param[in]    GMPPropPtr pProp
    *@param[in]    const GString& strNewValue
    *@param[in]    const GString& strOldValue
    */
    void WriteLeftSlopeFactor(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
    /*!
    * @brief �ҷ���ϵ��
    * @param[in] pProp ����
    * @param[in] strNewValue ������ʽ�ַ���
    */
    void WriteRightSlopeFactor(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);

    /*!
    *@brief  �޸Ĺ������
    *@author zhangh-t 2015��3��30��
    *@param[in]    GMPPropPtr pProp
    *@param[in]    const GString& strNewValue
    *@param[in]    const GString& strOldValue
    */
    void WriteWorkingFaceWidth(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue, bool bIsMirror = false);

    /*!
    *@brief
    *@author zhangh-t 2015��3��31��
    *@param[in]    GMPPropPtr pProp
    *@param[in]    const GString& strNewValue
    *@param[in]    const GString& strOldValue
    */
    //void WriteStartOrEndPtBottomHeight(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);

    void WriteStartPtBottomElev(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
    void WriteEndPtBottomElev(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);

private:
    bool m_bSuc;
};
/*!
*@file
*@brief    �������Ժ������������
*@author   yangwl-a
*@date     2015��4��2��
*@remarks
*@version 1.0
*Copyright (c) 1998-2013 Glodon Corporation
*/
class GTJNameAndTypePropInfoWriter :public GMPPropInfoDefaultWriter
{
public:
    GTJNameAndTypePropInfoWriter(std::vector<int> * pAcceptEntTypes = nullptr);
    ~GTJNameAndTypePropInfoWriter();

public:
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
    virtual bool validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg);

};

//���߾�����߾����ʵ���࣬��Ҫ�������������ʾ����ʵֵ��ͳһ�����⣬��Ҫ����һ����ʵֵ���д洢
class GTJCOMMON_EXPORT GTJAxisOffsetPropInfoWriter :public GMPPropInfoDefaultWriter
{
public:
    GTJAxisOffsetPropInfoWriter(std::vector<int>* pAcceptEntTypes = nullptr);
    ~GTJAxisOffsetPropInfoWriter();
public:
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
    virtual bool validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg);
    // �޸����߾�����߾���������ͨ��Ϣ����ȷ���������޸����߾�����߾�����ٴ�ͬ��һ��ǽ����ͨ
    void updateAllWallConnectionInfo(GMPPropPtr pProp);
    void updateEmbedBeamThroughWall(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
    void getEmbedBeam( GMPPropPtr pProp );
private:
    std::list<pair<IGMPElementDrawObj *, bool>> m_embedBeamList;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//��ʽ�����߿�Ĵ����������Ľ�����/ǽ�ĺ�ȵ�
class GTJCOMMON_EXPORT GTJLineWidthPropInfoWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJLineWidthPropInfoWriter(std::vector<int> *const pAcceptEntTypes = nullptr);
    virtual ~GTJLineWidthPropInfoWriter();
public:
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue) override;
public:
    virtual void writeElementProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
    virtual void writeEdoProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
    void resumeEdoDefaultValue(GMPPropPtr pProp, const GString& strNewValue);
    bool isChangeElementProp(GMPPropPtr pProp, double &dWidth);
    //����������޸ĵ������߾�����߾���Ƿ�ʱ���Ƿ���Ҫ����
    bool isNeedChangeAxialOffset(const int nElementType);

    double getCurrentValue(const QString &oValue, GMPPropPtr const pProp);
    //������������߾���ǽƤ��������ǽ��ȸı������
    void changeLintelAxisOffset(GMPPropPtr pProp);
    //�������ù���ͼԪ�������߾���ǽƤ����
    void setLintelAxisOffset(IGMPElementDrawObj* pLintelEdo, IGMPElementDrawObj * pParentEdo);
    void changeValideLintelAxisOffset(IGMPElementDrawObj *pTempEdo);
    // �޸�ǽ��������ͨ��Ϣ����ȷ���������޸�ǽ����ٴ�ͬ��һ������ǽ����ͨ
    void updateAllWallConnectionInfo(GMPPropPtr pProp);
protected:
    virtual bool defaultWrite(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class GTJCOMMON_EXPORT GTJAxisOffsetLineWidthProcessor
{
public:
    static GString calcRealAxisOffsetValue(GMPPropPtr pProp, const GString& strValue);
};
GTJCOMMON_EXPORT QString calcRealAxisOffset(GMPPropPtr const pProp, const QString &strValue);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ���ڱ���������ֶεĴ��������������Ľ����ȡ�����߶ȡ�����뾶
class GTJCOMMON_EXPORT GTJVariSecPropInfoWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJVariSecPropInfoWriter(vector<int> *const pAcceptEntTypes = nullptr);
public:
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue) override;
    virtual bool validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg) override;
private:
    bool validateBody(GMPPropPtr const pProp);
};

/**
* @brief ǽ��ȼ��ӣ���Ҫ�������������Ľ�����
*
* @class GCLWallThicknessListener
*/
struct NotifyLinkage
{
    int nLinkageType;
    int nLinkageEdoID;
    int nEdoID;
};
typedef NotifyLinkage *PNotifyLinkage;

//ltPropListener | ltEntListener | ltEdoListener
class GTJCOMMON_EXPORT GTJWallThicknessListener : public IGMPRelaDataListener
{
public:
    GTJWallThicknessListener() : m_iSectionWith(0), m_pEdoIterator(nullptr), m_pEdoContnr(nullptr) { }
    ~GTJWallThicknessListener() { }
    void onNotify(int iNotifyType, IGMPProperties* pProps, void* pData, bool bRedoUndo) override;
    void onNotify(int iNotifyType, IGMPElement* pEnt, void* pData, bool bUndoRedo);
    void onNotify(int iNotifyType, IGMPElementDrawObj* pEdo, void* pData, bool bUndoRedo);
    void onNotifyByGGJ(int iNotifyType, IGMPProperties* pProps, void* pData, bool bRedoUndo);
    virtual void onCalculate() override;
    virtual void onEndEdit();
private:
    void storeCorrelativeObjects(int nNotifyType, IGMPEObject* pUpdatedObject, IGMPPropertySchema* pPropSchema, bool bUndoRedo);
    void storeCorrelativeObjectsForSingleObject(int nFlag,
        IGMPElementDrawObj* pEDO, 
        CPositionJudge* pPositionJudge, 
        std::wstring& sPropName,
        IGMPEdoIterator* pIter);
    void calculateOrEndEdit();

private:
    int m_iSectionWith;
    IGMPEdoIterator *m_pEdoIterator;
    vector<NotifyLinkage> m_vecLinkage;
    IGMPElementDrawObjContnr * m_pEdoContnr;
    std::set<IGMPElementDrawObj*> m_setOperatedEDOs;
    std::map<IGMPElementDrawObj*, std::wstring> m_mapEDOModified;
    std::map<IGMPElement*, std::wstring> m_mapElementModified;
};

//������ǽɾ����Ԫ���߾�����߾��������
//ltPropListener
class GTJCOMMON_EXPORT GTJInsulatingWallThicknessListener : public IGMPRelaDataListener
{
public:
    GTJInsulatingWallThicknessListener() { }
    ~GTJInsulatingWallThicknessListener() { }
    void onNotify(int iNotifyType, IGMPProperties* pProps, void* pData, bool bRedoUndo) override;
private:
};


/*!
*@file
*@brief    ����ع���ɫ�浥Ԫλ�õ���������
*@author   zhangw-n
*@date     2016.5.24
*@remarks
*@version 1.0
*Copyright (c) 1998-2016 Glodon Corporation
*/
// ���ӵ�ordNum�����仯ʱ������ˢ���ӵ���ʾ״̬
//GTJUnitDisplayOrdNumSyncͳһ������
class GTJCOMMON_EXPORT GTJTrenchPropListener : public IGMPRelaDataListener
{
public:
    GTJTrenchPropListener(IGMPService* pService) : m_pService(pService){}
    virtual void onNotify(int iNotifyType, IGMPProperties* pProps, void* pData, bool bRedoUndo) override;
    virtual void onNotify(int iNotifyType, IGMPElement* pEnt, void* pData, bool bRedoUndo);
    virtual void onEndEdit();

private:
    IGMPService* m_pService;
    std::map<IGMPElement*, int> m_unitOrdNumMap;
    std::set<IGMPElement*> m_setOrdNumChangedElements;
};

/*!
*@brief  ������״���㰴ť����
*@author hanhc
*@return
*/
class  GTJCOMMON_EXPORT GTJParamSectionPropInfoWriter : public GTJParamNAbnormalWriter
{
public:
    GTJParamSectionPropInfoWriter(std::vector<int> *const pAcceptEntTypes = nullptr);
public:
    virtual bool validateProp(GMPPropPtr pProp, const GString &strValue, GString &strErrMsg) override;
    virtual void onButtonClick(std::vector<GMPPropPtr> &propVec, bool &modified) override;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ���޸����ӵĵױ��ʱ��ͬ���޸�������ϵ
class GTJCOMMON_EXPORT GTJColumnElevPropInfoWriter : public GMPElevPropInfoWriter
{
public:
    GTJColumnElevPropInfoWriter(std::vector<int> *const pAcceptEntTypes = nullptr);
public:
    virtual void writeProp(GMPPropPtr pProp, const GString &strNewValue, const GString &strOldValue) override;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class GTJCOMMON_EXPORT GTJArchBeamElevPropInfoWriter :public GMPPropInfoDefaultWriter
{
public:
    GTJArchBeamElevPropInfoWriter(IGMPService *const pService, std::vector<int> *const pAcceptEntTypes = nullptr);

public:
    virtual void writeProp(GMPPropPtr pProp, const GString &strNewValue, const GString &strOldValue) override;
    virtual bool validateProp(GMPPropPtr pProp, const GString &strNewValue, GString &strErrMsg) override;
    virtual bool beforeWriteProp(vector<GMPPropPtr>& oProps, const GString& strValue, GString& strErrMsg) override;
    virtual bool afterWriteProp(vector<GMPPropPtr>& oProps, const GString& strValue, GString& strErrMsg) override;
private:
    void updateElevOfCommonArchBeam(IGMPElementDrawObj *const pElmDrawObj);
    // ���޸İ빰�Ĺ����ı��ʱ����Ҫ�������ù��Ĳ���
    void updateSemiArchConfig(IGMPElementDrawObj *const pElmDrawObj, const std::wstring &pPropName);
    /*!
    *@brief ����д�루�̳л��෽����
    *@author yangwl-a 2015��6��11��
    */
    void writeLaiPlaneProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);

    void refreshBeamHanger(IGMPService *const pService, IGMPEObject *const pObject) const;
    bool getEdoElev( GMPPropPtr pProp, GString strNewValue, GString strOldValue, bool& bStatus, double& dNewElev, double& dOldElev );
    void writeElevProp(GMPPropPtr pProp, const GString &strNewValue, const GString &strOldValue);
    void writeFDBeamElevProp(GMPPropPtr pProp, const GString &strNewValue, const GString &strOldValue);
private:
   
     void writeLinkageProp(IGMPElementDrawObj *pEdo, GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
     void writeCommonProp(IGMPElementDrawObj *pEdo, GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
     /*!
     *@brief �ж��Ƿ�Ϊ��άƽ�У���ά��ƽ�е������������������ߣ�
     *@author yangwl-a 2015��6��11��
     */
     bool isPlaneBeam(IGMPElementDrawObj *const pEdo);
     bool isParallelBeam(IGMPElementDrawObj *pEdo);
     bool isLinkageBeam(IGMPElementDrawObj * pEdo);
 
     bool isAllLine2dBeam(IGMPElementDrawObj * pEdo);
     /*!
     *@brief ����������
     *@author yangwl-a 2015��6��11��
     */
     void calcBeamElve(GMPPropPtr pProp, IGMPElementDrawObj * pEdo, const GString& strNewValue,
         const GString& strOldValue);
     /*!
     *@brief ��������߷����仯���������������
     *@author yangwl-a 2015��6��11��
     */
     void calcNormalBeamElve(GMPPropPtr pProp, IGMPElementDrawObj *pEdo,
         double dLength, double dLengthNext, double dOldElve, double dNewElve);
     /*!
     *@brief �����յ��߷����仯���������������
     *@author yangwl-a 2015��6��11��
     */
     void calcSpecialBeamElve(GMPPropPtr pProp, IGMPElementDrawObj *pEdo,
         double dLength, double dLengthNext, double dOldElve, double dNewElve);
     /*!
     *@brief �����̨��������
     *@author yangwl-a 2015��6��11��
     */
     void calcFDBeamElve(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
 
     /*! �������˵Ҫ��ĺ���һ��б����
     *@brief �����̨���ͻ�������εı��
     *@author yangr-c 2015��12��31��
     */
     void calcLinkFDBeamElve(GMPPropPtr pProp, IGMPElementDrawObj * pEdo, const GString& strNewValue, const GString& strOldValue);
 
         /*!
     *@brief ��������߷����仯�����ӻ������������
     *@author yangwl-a 2015��12��31��
     */
     void calcNormalFDBeamElve(GMPPropPtr pProp, IGMPElementDrawObj *pEdo,
         double dLength, double dLengthNext, double dOldElve, double dNewElve);
 
         /*!
     *@brief �����յ��߷����仯�����ӻ������������
     *@author yangr-c 2015��12��31��
     */
     void calcSpecialFDBeamElve(GMPPropPtr pProp, IGMPElementDrawObj *pEdo, double dLength,
         double dLengthNext, double dOldElve, double dNewElve);

private:
    IGMPService *m_pService;
    bool m_bElevChange;
    int m_pArchBeamCount;
    std::map<IGMPElementDrawObj*, pair<double, double>> m_mapEdoAndTopElevs;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class GTJCOMMON_EXPORT GTJFDBeamElevPropInfoWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJFDBeamElevPropInfoWriter(std::vector<int> *const pAcceptEntTypes = nullptr);
public:
    virtual void writeProp(GMPPropPtr pProp, const GString &strNewValue, const GString &strOldValue) override;
private:
    void configProp(GMPPropPtr const pProp, const GString &strNewValue, const GString &strOldValue);
    bool checkMultiSegments(IGMPElementDrawObj *const pEdo, const std::wstring &strStartPt,
        const std::wstring &strEndPt, std::vector<double> &lengthVec);
    void configSegmentsElev(IGMPRelationOperator *const pRelaOpr, IGMPElementDrawObj *const pEdo,
        const std::wstring &strStartPt, const std::wstring &strEndPt, const double dHeight,
        const std::vector<double> &lengthVec);
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
class GTJCOMMON_EXPORT GTJFDBeamSectionHeightPropInfoWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJFDBeamSectionHeightPropInfoWriter(std::vector<int> *const pAcceptEntTypes = nullptr);
public:
    virtual void writeProp(GMPPropPtr pProp, const GString &strNewValue, const GString &strOldValue) override;
};
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/*
 * @brief    �޸Ĺ����Ľ���߶Ƚ��кϷ���У��
 * @author   liuk-g
 * @date     2015��07��30��
 */
class GTJCOMMON_EXPORT GTJArchBeamSectionHeightWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJArchBeamSectionHeightWriter(std::vector<int> * pAcceptEntTypes = nullptr);
public:
    virtual void writeProp(GMPPropPtr pProp, const GString &strNewValue, const GString &strOldValue);
    virtual bool validateProp(GMPPropPtr pProp, const GString &strValue, GString &strErrMsg);
    bool beforeWriteProp(vector<GMPPropPtr>& oProps, const GString& strValue, GString& strErrMsg);
public:
    bool m_dIsAllPlain;
    bool m_dIsArchToVariable;
};

/////////////////////////��propinfoWriter end///////////////////////////////

/*
 * �������ֽ���ͨ�����Ե�writer
 */
//zhangh-t 2015��5��17��fixed bug TJGJ-18446 ���޸ı�ߵ�ʱ��б����ƽ�壬����й���Ϣ�Ļ��������ﴦ����Ϣ�ĸ���
class GTJCOMMON_EXPORT GTJSlabPropWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJSlabPropWriter(std::vector<int> * pAcceptEntTypes = nullptr);
    ~GTJSlabPropWriter() { }
public:
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
};

//������
class GTJCOMMON_EXPORT GTJSpiralSlabPropWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJSpiralSlabPropWriter(vector<int> *const pAcceptEntTypes = nullptr);
public:
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue) override;
    virtual bool validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg) override;
};

//���ʹ�
class GTJCOMMON_EXPORT GTJRibbonWindowPropWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJRibbonWindowPropWriter(vector<int> *const pAcceptEntTypes = nullptr);
public:
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue) override;
    virtual bool validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg) override;
};


class GTJCOMMON_EXPORT GTJSumpSectionWidthHeightPropInfoWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJSumpSectionWidthHeightPropInfoWriter(std::vector<int> *const pAcceptEntTypes = nullptr);
    ~GTJSumpSectionWidthHeightPropInfoWriter();
public:
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue) override;

private:
    /*����ˮ�ӵ�worldpoly���븸�ı��غ�ʱ�����ü�ˮ�ӿ�ͳ���Ӧʹ��ˮ��������չ���Ѿ�������ͼԪ���������Ƿ�
           |---|---------------|
           |---|               |
           |                   |
           |                   |
           |                   |
           |-------------------|

           ���ÿ���Ч��
           |-------|-----------|
           |-------|           |
           |                   |
           |                   |
           |                   |
           |-------------------|

           ���ó����Ч��
           |---|---------------|
           |   |               |
           |   |               |
           |---|               |
           |                   |
           |-------------------|
           */
    void AdjustSumpCoordinate(GMPPropPtr pProp, double dNewValue, double dOldValue);

    void AdjustEDO(IGMPElementDrawObj* pTmpEdo, GMPPropPtr pProp, double dNewValue, double dOldValue);

    bool TheCurveHasParallelsInVectorAndRemoveIt(CCurve2d* pSumpCurve, vector<CCurve2d*>& oSumpCurvesOnParentPoly);
    bool TheLineIsWidth(CLine2d* pLine, CCoordinates3d& oCoord);
    bool TheLineIsHeight(CLine2d* pLine, CCoordinates3d& oCoord);
};

class GTJCOMMON_EXPORT GTJStripFDUnitPropInfoWriter :public GMPPropInfoDefaultWriter
{
public:
    GTJStripFDUnitPropInfoWriter(std::vector<int> * pAcceptEntTypes = nullptr);
    ~GTJStripFDUnitPropInfoWriter() { }
public:
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
    virtual bool validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg);
    void changeEDOAxisOffset(IGMPEObject * pObj1, IGMPEObject * pObj2, double &dAxisOffset, bool isNull);
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ���ڴ�������ȵ���ʾ
class GTJCOMMON_EXPORT GTJEmbedBeamPropWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJEmbedBeamPropWriter(vector<int> *const pAcceptEntTypes = nullptr);
public:
    virtual void writeProp(GMPPropPtr pProp, const GString &strNewValue, const GString &strOldValue) override;
    virtual bool validateProp(GMPPropPtr pProp, const GString &strValue, GString &strErrMsg) override;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//ֱ���ݶ�
class GTJCOMMON_EXPORT GTJFlightPropWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJFlightPropWriter(vector<int> *const pAcceptEntTypes = nullptr);
public:
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue) override;
    virtual bool validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg) override;
};

//�����ݶ�
class GTJCOMMON_EXPORT GTJSpiralFlightPropWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJSpiralFlightPropWriter(vector<int> *const pAcceptEntTypes = nullptr);
public:
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue) override;
    virtual bool validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg) override;
};

/*!
*@brief    ����ع��ױ�ߺ����ױ�ߣ��յ�ױ�ߵ���������
*@author   wangf-h
*@date     2014��12��5��
*/
class GTJCOMMON_EXPORT GTJTrenchPropInfoWriter :public GMPPropInfoDefaultWriter
{
public:
    GTJTrenchPropInfoWriter(IGMPService *const pService, std::vector< int > * pAcceptEntTypes = nullptr);
public:
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
    virtual void onButtonClick(vector<GMPPropPtr>& oProps, bool& bModify);

private:
    IGMPElement *getElement(GMPPropPtr pProperty);
    bool isSecInfoPropDiff(const vector<GMPPropPtr>& oProps);
    void getOrgElement(const vector<GMPPropPtr>& oProps, set<IGMPElement *>&vOrgElements);

private:
    IGMPService* m_pService;

};


// ���ڴ�����ʽ����Ⱥ���ʵ���֮�����������
class GTJCOMMON_EXPORT GTJBeddingSectionHeightPropInfoWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJBeddingSectionHeightPropInfoWriter(vector<int> *const pAcceptEntTypes = nullptr);
public:
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue) override;
    virtual bool validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg);
    void setPropValue(IGMPElement * pElement, const GString cStrValue);
public:
    virtual bool afterWriteProp(vector<GMPPropPtr>& oProps, const GString& strValue, GString& strErrMsg)
    {
        m_setModifiedElements.clear();
        return true;
    }
private:
    //added by zhangh-t ���޸Ĺ��Ĺ������ԣ���ֹ�ظ��޸�
    std::set<IGMPElement*> m_setModifiedElements;
};


/*
 * @brief    �޸�̨��̤���������кϷ���У��
 * @author   lixd-a
 * @date     2015��08��10��
 */
class GTJCOMMON_EXPORT GTJFootStepInfoWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJFootStepInfoWriter(std::vector<int> * pAcceptEntTypes = nullptr);
public:
    virtual bool validateProp(GMPPropPtr pProp, const GString &strValue, GString &strErrMsg);
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////
/*!
*@brief    �˴�����ǽ��writerר�Ŵ����ֶ��޸�����ǽ��ʾ��ʽʱ����һ�¼��������ڲ�������ɫ����ʱ�ж��Ƿ�ΪĬ��ֵ�Ĵ���
*@author   zhaojs
*@date     2016.3.21
*@remarks
*@version 1.0
*Copyright (c) 1998-2013 Glodon Corporation
*/
class GTJCOMMON_EXPORT GTJBrickWallDFStylePropWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJBrickWallDFStylePropWriter(vector<int> *const pAcceptEntTypes = nullptr);
    ~GTJBrickWallDFStylePropWriter(){ }

    void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue) override;
};

///////////////////////////////////////////////////////////////////////////////////////////////////////////
/*!
*@brief    ������¥���޸Ĺ�������
*@author   hesp
*@date     2016.4.20
*@remarks
*@version 1.0
*Copyright (c) 1998-2013 Glodon Corporation
*/
class GTJCOMMON_EXPORT GTJStairPropWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJStairPropWriter(vector<int> *const pAcceptEntTypes = nullptr);
    ~GTJStairPropWriter(){ }

    void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue) override;
};

class GTJCOMMON_EXPORT GTJPostCastElevPropInfoWriter :public GMPElevPropInfoWriter
{
public:
    GTJPostCastElevPropInfoWriter(vector<int> *pAcceptEntTypes = nullptr);
};

/*!
*@brief  �˜ʈD��
*@author zhangw-n
*@return
*/
class  GTJCOMMON_EXPORT GTJDrawingSetPropInfoWriter : public GMPPropInfoWriter
{
public:
    GTJDrawingSetPropInfoWriter(std::vector<int> *const pAcceptEntTypes = nullptr);
public:
    virtual void onButtonClick(std::vector<GMPPropPtr> &propVec, bool &modified);

    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);	
    virtual bool validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg);

    virtual int getElementType() { return -1; }
    virtual GString getElementTableName() { return ""; }
    virtual bool saveProperties(IGMPProperties* pProps, GSPRecord pRecord) { return false; }

    GString normalizeCode(const GString &sCode);
private:	
};

// ��
class  GTJCOMMON_EXPORT GTJDrawingSetDoorPropInfoWriter : public GTJDrawingSetPropInfoWriter
{
public:
    GTJDrawingSetDoorPropInfoWriter(std::vector<int> *const pAcceptEntTypes = nullptr) : GTJDrawingSetPropInfoWriter(pAcceptEntTypes){}

    virtual int getElementType() { return etDoor; }
    virtual GString getElementTableName() { return c_strTableDrawingDoor; }
    virtual bool saveProperties(IGMPProperties* pProps, GSPRecord pRecord);

private:	
};

// ��
class  GTJCOMMON_EXPORT GTJDrawingSetWindowPropInfoWriter : public GTJDrawingSetPropInfoWriter
{
public:
    GTJDrawingSetWindowPropInfoWriter(std::vector<int> *const pAcceptEntTypes = nullptr) : GTJDrawingSetPropInfoWriter(pAcceptEntTypes){}

    virtual int getElementType() { return etWindow; }
    virtual GString getElementTableName() { return c_strTableDrawingWindow; }
    virtual bool saveProperties(IGMPProperties* pProps, GSPRecord pRecord);

private:	
};

// ������
class  GTJCOMMON_EXPORT GTJDrawingSetDoorWinPropInfoWriter : public GTJDrawingSetPropInfoWriter
{
public:
    GTJDrawingSetDoorWinPropInfoWriter(std::vector<int> *const pAcceptEntTypes = nullptr) : GTJDrawingSetPropInfoWriter(pAcceptEntTypes){}

    virtual int getElementType() { return etDoorWin; }
    virtual GString getElementTableName() { return c_strTableDrawingDoorWin; }
    virtual bool saveProperties(IGMPProperties* pProps, GSPRecord pRecord);

private:	
};

// ����
class GTJCOMMON_EXPORT GTJDrawingSetLintelPropInfoWriter : public GTJDrawingSetPropInfoWriter
{
public:
    GTJDrawingSetLintelPropInfoWriter(std::vector<int> *const pAcceptEntTypes = nullptr) : GTJDrawingSetPropInfoWriter(pAcceptEntTypes){}

    virtual int getElementType() { return etLintel; }
    virtual GString getElementTableName() { return c_strTableDrawingLintel; }
    virtual bool saveProperties(IGMPProperties* pProps, GSPRecord pRecord);

private:	
};

class GTJSensorConfigWidget;
class GTJCOMMON_EXPORT GTJCustomLineObstacleTagWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJCustomLineObstacleTagWriter(vector<int> * pAcceptEntTypes = nullptr);
    ~GTJCustomLineObstacleTagWriter() {}
public:
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
private:
    GTJSensorConfigWidget* createSensorInfoWidget();
    bool setObstacle(IGMPElementDrawObj* pEDO);
    void cancleObstatle(IGMPElementDrawObj* pEDO);
};

#endif //GTJPROPINFOWRITER_H
