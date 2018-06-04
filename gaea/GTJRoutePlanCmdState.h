#pragma once
#include "GMPCore/GMPCmdStateIntf.h"
#include <QMap>
#include <QList>
#include <QLabel>
#include <QPainter>
#include <QTimer>
#include <QSet>
#include "GTJRoutePlan.h"
#include "qrencode.h"
#include "common/IObserver.h"
#include <functional>

class IGMPCustomLineSolidShape;
class GTJQRCode;
class GTJQRWidget;
class GTJRoutePlanWebCom;
class GTJRoutePlanCmdState : public IGMPBaseCmdState
{
private:
    struct obstacleCutParam
    {
        double dT;
        double dLength;
        bool operator < (const obstacleCutParam& other) const;
    };
public:
    GTJRoutePlanCmdState(void);
    ~GTJRoutePlanCmdState(void);
public:
    virtual void exec();
    virtual bool initialCustomData(int nUserData, void *pUserData) { return true; }
private:
    void acquireAllEmergencyExist(QMap<IGMPElementDrawObj*, QList<IGMPElementDrawObj*>>& oExit,
        GTJRoutePlanStartEndPoint& oStartEndPoint,
        IGMPElementDrawObj* pStartEDO);
    void addEmergencyExit(IGMPElementDrawObj* pParent, IGMPElementDrawObj* pDoor,
        QMap<IGMPElementDrawObj*, QList<IGMPElementDrawObj*>>& oExit);
    void breakObstacle(IGMPCustomLineSolidShape* pShape,
        QList<ggp::CBodyPtr>& oObstacles,
        QList<ggp::CCurve2dPtr>& oCurves,
        QMap<IGMPElementDrawObj*, QList<IGMPElementDrawObj*>>& oExit);
    void breakLineObstacle(ggp::CCurve2dPtr pWorldLine,
        ggp::CBodyPtr pBody,
        std::set<obstacleCutParam> oCutParams,
        QList<ggp::CBodyPtr>& oObstacles,
        QList<ggp::CCurve2dPtr>& oCurves);
    void breakOtherObstacle(ggp::CCurve2dPtr pWorldLine,
        ggp::CBodyPtr pBody,
        std::set<obstacleCutParam> oCutParams,
        QList<ggp::CBodyPtr>& oObstacles,
        QList<ggp::CCurve2dPtr>& oCurves);
    void exportAndFeedBack(ggp::CViewer* pViewer,
        QString& oFeedBackID,
        ggp::CVector3d& oPosition,
        ggp::CVector3d& oDir);
private:
    QList<ggp::CBodyPtr> acquireFireObstale(int64_t nFloorID, QSet<int>& oSensorState);
    void initialize(QMap<IGMPElementDrawObj*, QList<IGMPElementDrawObj*>>& oExit,
        GTJRoutePlanStartEndPoint& oStartEndPoint,
        IGMPElementDrawObj* pStartEDO);
    void finalize();
    void onRequest(int64_t nPositionID, int64_t nFloorID, QString& oFeedBackID);
    void showQR(int64_t nFloorID, 
        int64_t nPosID,
        ggp::CVector2i& pos,
        ggp::CVector3d& oAnchor);
private:
    bool getPositionAndExit(int64_t nPositionID, 
        GTJRoutePlanStartEndPoint& oStartEndInfo,
        IGMPElementDrawObj*& pStartEDO,
        QMap<IGMPElementDrawObj*, QList<IGMPElementDrawObj*>>& oExit);
    void onRoutePlanRequest(QString& strRequestParam);
    QSet<int> parseSensorState(const QString& strSensorStateInHex);
private:
    QMap<IGMPElementDrawObj*, QList<IGMPElementDrawObj*>> m_oEmergencyExists;
    GTJRouteplanModel* m_pRoutePlaneModel;
    GTJQRCode* m_pQRCode;
private:
    GTJQRWidget* m_qrWidget;
    GTJRoutePlanWebCom* m_pCom;
};


class GTJQRCode : public QLabel
{
    Q_OBJECT
public:
    GTJQRCode();
    ~GTJQRCode();
public:
    void refresh(QString& sProjName,
        int64_t nFloorID,
        int64_t nPosID);
protected:
    QSize sizeHint();
    QSize minimumSize();
    void paintEvent(QPaintEvent * e);
private:
    void drawQR(QPainter& painter, int width, int height);
private:
    QRcode* m_code;
};

class GTJQRWidget : public QWidget
{
    Q_OBJECT
public:
    GTJQRWidget(QWidget* parent = 0);
    ~GTJQRWidget();
public:
    ggp::CVector3d m_oAnchor;
public:
    void showQR(QString& sBuilding, 
        int64_t nFloorID, 
        int64_t nPosID,
        ggp::CVector3d& oAnchor);
    void destroy();
    void updatePos(ggp::CVector2i oPosOnParent);
private:
    void init();
private:
    GTJQRCode* m_qrCode;
    QWidget* m_parent;

};

class GTJQRWidgetObserver : public ggp::IObserver
{
public:
    GTJQRWidgetObserver(GTJQRWidget* pControl,
        ggp::CViewer* pViewer);
public:
    virtual void Update(unsigned int unEvent, void* pEventData);
private:
    GTJQRWidget* m_pControl;
    ggp::CViewer* m_pViewer;
};

typedef std::function<void (QString)> GTJRoutePlanRequestCallBack;
class GTJRoutePlanWebCom : public QObject
{
    Q_OBJECT
public:
    GTJRoutePlanWebCom(GTJRoutePlanRequestCallBack& oCallBack);
    ~GTJRoutePlanWebCom();
public:
    void post(QJsonObject& data);
    QString sensorState();
public slots:
    void onRequest(const QString& strRequestParam);
public:
    GTJRoutePlanRequestCallBack m_callBack;
};