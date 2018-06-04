#ifndef GTFWEBGAEAINTERFACE_H
#define GTFWEBGAEAINTERFACE_H

#include <QString>
#include <QJsonObject>
#include <QObject>
#ifdef _DLL_SAMPLE  
#define DLL_SAMPLE_API __declspec(dllexport)  
#else  
#define DLL_SAMPLE_API __declspec(dllimport)  
#endif 

class ICommunicate : public QObject
{
    Q_OBJECT
public:
    virtual void config(QJsonObject& config) = 0;
    virtual void getURLfromWeb(const QString& strURL) = 0;
    virtual QString getSensorState(void) = 0;
    virtual void post(const QJsonObject& oPostData) = 0;
Q_SIGNALS:
    void routePlanRequest(const QString& sRequestParam);
public:
    virtual ~ICommunicate(void){}
};

#ifdef __cplusplus
extern "C"
{
#endif

DLL_SAMPLE_API ICommunicate* getGAEAInterface(void);

#ifdef __cplusplus
}
#endif

#endif
