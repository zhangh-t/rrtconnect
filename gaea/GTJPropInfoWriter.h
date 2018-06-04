/*!
*@file      GGJPropInfoWriter.h
*@brief     产品特有属性处理
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
    //added by zhangh-t 解决桩承台单元重复遍历的效率问题
    virtual void afterModifySectionInfo(vector<GMPPropPtr>& oProps);
    bool isModified(IGMPEObject* pObj);
    std::set<IGMPElement*> m_setModifiedElements;
};

/**
* @brief 专门处理复杂构件的标高属性联动（比如桩承台 独立基础等）
*原则 高度尽量保持不变
*如果修改顶标高 则同时修改低标高
*如果修改底标高 则同时修改顶标高
*如果修改高度 则同时修改顶标高
*
* @remark 注册在 GGJUIPlugin::registerPropWriters
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
* @brief 专门处理复杂构件(如桩承台单元)的填充颜色、边框颜色、不透明度的属性联动
* 原则：(1) 父构件属性改变，单元构件随父构件改变(前提是单元构件属性为空值)
*       (2) 单元构件属性改变，不影响父构件属性
*       (3) 修改父时，子是空值时，子本身就会联运
*       (4) 修改子时，判断如果和父一样  就变为空值
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
* @brief 专门处理具有（顶标高 底标高 和高度）的构件的标高属性三联动问题
* 原则 厚度尽量保持不变
* 厚度->顶
* 顶->底
* 底->顶
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
*@brief  参数化墙三点按钮解析
*@author wangxb 2014年12月5日
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
* @brief 专门处理复杂构件的父构件的高度属性（比如桩承台 独立基础等）
*
* @class GGJComplexHeightLiserner
*/
//ltPropListener | ltEntListener | ltEdoListener
class GTJCOMMON_EXPORT GTJComplexHeightListener : public IGMPRelaDataListener
{
public:
    GTJComplexHeightListener(IGMPService *pIGMPService);
    //开始批量修改通知
    virtual void onBeginEdit();
    //结束批量修改通知
    virtual void onEndEdit();

    virtual void onNotify(int iNotifyType, IGMPProperties* pProps, void* pData, bool bRedoUndo);
    //构件
    virtual void onNotify(int iNotifyType, IGMPElement* pEnt, void* pData, bool bRedoUndo);
    //图元
    virtual void onNotify(int iNotifyType, IGMPElementDrawObj* pEdo, void* pData, bool bRedoUndo);
private:
    std::map<IGMPProperties*, IGMPPropertySchema*> m_oMap;
    IGMPService     *m_pIGMPService;
};

////////////////////////////////////////类似柱帽或柱墩“类型”处理/////////////////////////////////////
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

/////////////////////////////////////后浇带参数类型处理/////////////////////////////////////////////////
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
*@brief    处理过梁的属性“方位”与“标高”联动
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
        LINETEL_HOLD_UP = 0, // 洞口上方
        LINETEL_HOLD_DOWN = 1 // 洞口下方
    };

private:
    // 带拱的过梁，其标高修改的处理
    bool updateElevOfArch(GMPPropPtr const pProp, const GString &strNewValue, const GString &strOldValue);
    // 判断修改起/始点深入墙的距离后是否超过圆周长
    bool isOverCircumference(CCurve2d* pLine, int nPtLenInWall = 250, int nNewValue = 250);
     //过梁中心线距左墙皮距离的合法性校验
    bool isValidAxisOffset(IGMPElementDrawObj* pEdo, double& dThickness, double& dCenterAxisOffset, GString& strErrMsg);
    //获取过梁的父相交的墙，并进行排序
    void getWallByLintelParent(vector<IGMPElementDrawObj*> &oWallVector, IGMPElementDrawObj *pEDO, IGMPElementDrawObj *pParentEdo);
};

/**
* @brief 处理集水坑放坡后，通过在属性框修改边坡信息，统一修改所有边的边坡信息
* 原则 设置出边距离，放坡底宽/放坡角度
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
*@brief    柱墩属性联动
*@author   zoul-a
*@date     2015年1月19日
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
*@brief    钢筋柱类截面需另处理
*@author   qinyb
*@date     2015年1月21日
*Copyright (c) 1998-2015 Glodon Corporation
*/
////////////////////////////////////////////截面宽属性设值类////////////////////////////////////////////
class GTJCOMMON_EXPORT GTJSectionWidthWriter : public GMPSectionWidthWriter
{
public:
    GTJSectionWidthWriter(vector<int> * pAcceptEntTypes = nullptr);
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
};

////////////////////////////////////////////截面高属性设值类////////////////////////////////////////////
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
*@brief    处理基槽回填属性
*@author   zhangh-t
*@date     2015年3月27日
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
            // 修改原因：当修改的槽底宽为空，还原后的默认值小于轴线距左边线距离时
            //           应该不容许修改，提示用户重新输入
            if (strValue.isEmpty())
            {
                // 对属性值进行单位化
                GString strDefaultBottomWidth;

                IGMPEObject* pObject = pProp->owner()->owner();
                strDefaultBottomWidth = pProp->schema()->defaultExpr();
                // 验证转换后的值是否合法
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
    *@brief  处理放坡系数表达式
    *@author zhangh-t 2015年3月27日
    *@param[in]    const QString& strNewValue
    *@return       QString
    */
    QString handleValue(const QString& strNewValue);

    /*!
    *@brief  处理200.00这种整数
    *@author zhangh-t 2015年3月27日
    *@param[in]    const QString& strNewValue
    *@return       QString
    */
    QString handleInteger(const QString& strNewValue);

    /*!
    *@brief  左放坡系数
    *@author zhangh-t 2015年3月27日
    *@param[in]    GMPPropPtr pProp
    *@param[in]    const GString& strNewValue
    *@param[in]    const GString& strOldValue
    */
    void WriteLeftSlopeFactor(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
    /*!
    * @brief 右放坡系数
    * @param[in] pProp 属性
    * @param[in] strNewValue 输入表达式字符串
    */
    void WriteRightSlopeFactor(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);

    /*!
    *@brief  修改工作面宽
    *@author zhangh-t 2015年3月30日
    *@param[in]    GMPPropPtr pProp
    *@param[in]    const GString& strNewValue
    *@param[in]    const GString& strOldValue
    */
    void WriteWorkingFaceWidth(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue, bool bIsMirror = false);

    /*!
    *@brief
    *@author zhangh-t 2015年3月31日
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
*@brief    处理属性和类别联动问题
*@author   yangwl-a
*@date     2015年4月2日
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

//轴线距左边线距离的实现类，主要处理镜像后轴线显示与真实值不统一的问题，需要计算一下真实值进行存储
class GTJCOMMON_EXPORT GTJAxisOffsetPropInfoWriter :public GMPPropInfoDefaultWriter
{
public:
    GTJAxisOffsetPropInfoWriter(std::vector<int>* pAcceptEntTypes = nullptr);
    ~GTJAxisOffsetPropInfoWriter();
public:
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
    virtual bool validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg);
    // 修改轴线距左边线距离会造成拉通信息不正确，这里在修改轴线距左边线距离后再次同步一下墙的拉通
    void updateAllWallConnectionInfo(GMPPropPtr pProp);
    void updateEmbedBeamThroughWall(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
    void getEmbedBeam( GMPPropPtr pProp );
private:
    std::list<pair<IGMPElementDrawObj *, bool>> m_embedBeamList;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//线式构件线宽的处理，比如梁的截面宽度/墙的厚度等
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
    //当构件框厚修改导致轴线距左边线距离非法时，是否需要联动
    bool isNeedChangeAxialOffset(const int nElementType);

    double getCurrentValue(const QString &oValue, GMPPropPtr const pProp);
    //处理过梁中心线距左墙皮距离随着墙厚等改变的联动
    void changeLintelAxisOffset(GMPPropPtr pProp);
    //重新设置过梁图元的中心线距左墙皮距离
    void setLintelAxisOffset(IGMPElementDrawObj* pLintelEdo, IGMPElementDrawObj * pParentEdo);
    void changeValideLintelAxisOffset(IGMPElementDrawObj *pTempEdo);
    // 修改墙厚会造成拉通信息不正确，这里在修改墙厚后再次同步一下所有墙的拉通
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
// 用于变截面属性字段的处理，例如变截面柱的截面宽度、截面高度、截面半径
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
* @brief 墙厚度监视，主要用来联动连梁的截面宽度
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

//处理保温墙删除单元轴线距左边线距离的联动
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
*@brief    处理地沟颜色随单元位置调整的联动
*@author   zhangw-n
*@date     2016.5.24
*@remarks
*@version 1.0
*Copyright (c) 1998-2016 Glodon Corporation
*/
// 在子的ordNum发生变化时，触发刷新子的显示状态
//GTJUnitDisplayOrdNumSync统一处理了
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
*@brief  截面形状三点按钮解析
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
// 当修改柱子的底标高时，同步修改其坐标系
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
    // 当修改半拱的拱梁的标高时，需要重新设置拱的参数
    void updateSemiArchConfig(IGMPElementDrawObj *const pElmDrawObj, const std::wstring &pPropName);
    /*!
    *@brief 属性写入（继承基类方法）
    *@author yangwl-a 2015年6月11日
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
     *@brief 判断是否为三维平行，二维不平行的连接梁（不包括弧线）
     *@author yangwl-a 2015年6月11日
     */
     bool isPlaneBeam(IGMPElementDrawObj *const pEdo);
     bool isParallelBeam(IGMPElementDrawObj *pEdo);
     bool isLinkageBeam(IGMPElementDrawObj * pEdo);
 
     bool isAllLine2dBeam(IGMPElementDrawObj * pEdo);
     /*!
     *@brief 计算标高属性
     *@author yangwl-a 2015年6月11日
     */
     void calcBeamElve(GMPPropPtr pProp, IGMPElementDrawObj * pEdo, const GString& strNewValue,
         const GString& strOldValue);
     /*!
     *@brief 设置起点标高发生变化，连接梁标高联动
     *@author yangwl-a 2015年6月11日
     */
     void calcNormalBeamElve(GMPPropPtr pProp, IGMPElementDrawObj *pEdo,
         double dLength, double dLengthNext, double dOldElve, double dNewElve);
     /*!
     *@brief 设置终点标高发生变化，连接梁标高联动
     *@author yangwl-a 2015年6月11日
     */
     void calcSpecialBeamElve(GMPPropPtr pProp, IGMPElementDrawObj *pEdo,
         double dLength, double dLengthNext, double dOldElve, double dNewElve);
     /*!
     *@brief 处理承台梁特殊标高
     *@author yangwl-a 2015年6月11日
     */
     void calcFDBeamElve(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
 
     /*! 需求左宏说要变的和梁一样斜起来
     *@brief 处理承台梁和基础梁多段的标高
     *@author yangr-c 2015年12月31日
     */
     void calcLinkFDBeamElve(GMPPropPtr pProp, IGMPElementDrawObj * pEdo, const GString& strNewValue, const GString& strOldValue);
 
         /*!
     *@brief 设置起点标高发生变化，连接基础梁标高联动
     *@author yangwl-a 2015年12月31日
     */
     void calcNormalFDBeamElve(GMPPropPtr pProp, IGMPElementDrawObj *pEdo,
         double dLength, double dLengthNext, double dOldElve, double dNewElve);
 
         /*!
     *@brief 设置终点标高发生变化，连接基础梁标高联动
     *@author yangr-c 2015年12月31日
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
 * @brief    修改拱梁的截面高度进行合法性校验
 * @author   liuk-g
 * @date     2015年07月30日
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

/////////////////////////梁propinfoWriter end///////////////////////////////

/*
 * 处理下现浇板通用属性的writer
 */
//zhangh-t 2015年5月17日fixed bug TJGJ-18446 在修改标高的时候，斜板会变平板，如果有拱信息的话，在这里处理拱信息的更新
class GTJCOMMON_EXPORT GTJSlabPropWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJSlabPropWriter(std::vector<int> * pAcceptEntTypes = nullptr);
    ~GTJSlabPropWriter() { }
public:
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue);
};

//螺旋板
class GTJCOMMON_EXPORT GTJSpiralSlabPropWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJSpiralSlabPropWriter(vector<int> *const pAcceptEntTypes = nullptr);
public:
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue) override;
    virtual bool validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg) override;
};

//带型窗
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
    /*当集水坑的worldpoly边与父的边重合时，设置集水坑宽和长，应使集水坑向内扩展，已尽量避免图元超出父而非法
           |---|---------------|
           |---|               |
           |                   |
           |                   |
           |                   |
           |-------------------|

           设置宽后的效果
           |-------|-----------|
           |-------|           |
           |                   |
           |                   |
           |                   |
           |-------------------|

           设置长后的效果
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
// 用于处理暗梁宽度的显示
class GTJCOMMON_EXPORT GTJEmbedBeamPropWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJEmbedBeamPropWriter(vector<int> *const pAcceptEntTypes = nullptr);
public:
    virtual void writeProp(GMPPropPtr pProp, const GString &strNewValue, const GString &strOldValue) override;
    virtual bool validateProp(GMPPropPtr pProp, const GString &strValue, GString &strErrMsg) override;
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//直形梯段
class GTJCOMMON_EXPORT GTJFlightPropWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJFlightPropWriter(vector<int> *const pAcceptEntTypes = nullptr);
public:
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue) override;
    virtual bool validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg) override;
};

//螺旋梯段
class GTJCOMMON_EXPORT GTJSpiralFlightPropWriter : public GMPPropInfoDefaultWriter
{
public:
    GTJSpiralFlightPropWriter(vector<int> *const pAcceptEntTypes = nullptr);
public:
    virtual void writeProp(GMPPropPtr pProp, const GString& strNewValue, const GString& strOldValue) override;
    virtual bool validateProp(GMPPropPtr pProp, const GString& strValue, GString& strErrMsg) override;
};

/*!
*@brief    处理地沟底标高和起点底标高，终点底标高的联动问题
*@author   wangf-h
*@date     2014年12月5日
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


// 用于处理线式垫层宽度和真实宽度之间的联动问题
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
    //added by zhangh-t 已修改过的公有属性，防止重复修改
    std::set<IGMPElement*> m_setModifiedElements;
};


/*
 * @brief    修改台阶踏步个数进行合法性校验
 * @author   lixd-a
 * @date     2015年08月10日
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
*@brief    此处砌体墙的writer专门处理手动修改砌体墙显示样式时进行一下监听，用于材质与颜色联动时判断是否为默认值的处理
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
*@brief    参数化楼梯修改构件名称
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
*@brief  標準圖集
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

// 门
class  GTJCOMMON_EXPORT GTJDrawingSetDoorPropInfoWriter : public GTJDrawingSetPropInfoWriter
{
public:
    GTJDrawingSetDoorPropInfoWriter(std::vector<int> *const pAcceptEntTypes = nullptr) : GTJDrawingSetPropInfoWriter(pAcceptEntTypes){}

    virtual int getElementType() { return etDoor; }
    virtual GString getElementTableName() { return c_strTableDrawingDoor; }
    virtual bool saveProperties(IGMPProperties* pProps, GSPRecord pRecord);

private:	
};

// 窗
class  GTJCOMMON_EXPORT GTJDrawingSetWindowPropInfoWriter : public GTJDrawingSetPropInfoWriter
{
public:
    GTJDrawingSetWindowPropInfoWriter(std::vector<int> *const pAcceptEntTypes = nullptr) : GTJDrawingSetPropInfoWriter(pAcceptEntTypes){}

    virtual int getElementType() { return etWindow; }
    virtual GString getElementTableName() { return c_strTableDrawingWindow; }
    virtual bool saveProperties(IGMPProperties* pProps, GSPRecord pRecord);

private:	
};

// 门联窗
class  GTJCOMMON_EXPORT GTJDrawingSetDoorWinPropInfoWriter : public GTJDrawingSetPropInfoWriter
{
public:
    GTJDrawingSetDoorWinPropInfoWriter(std::vector<int> *const pAcceptEntTypes = nullptr) : GTJDrawingSetPropInfoWriter(pAcceptEntTypes){}

    virtual int getElementType() { return etDoorWin; }
    virtual GString getElementTableName() { return c_strTableDrawingDoorWin; }
    virtual bool saveProperties(IGMPProperties* pProps, GSPRecord pRecord);

private:	
};

// 过梁
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
