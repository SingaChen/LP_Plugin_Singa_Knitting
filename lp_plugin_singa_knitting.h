#ifndef LP_PLUGIN_SINGA_KNITTING_H
#define LP_PLUGIN_SINGA_KNITTING_H

#include "LP_Plugin_Singa_Knitting_global.h"
#include "plugin/lp_actionplugin.h"

#include "extern/geodesic/geodesic_algorithm_exact.h"
#include <QReadWriteLock>
#include <QThreadPool>
#include <QVector3D>
#include <QVector2D>
#include <QWaitCondition>
#include <QFutureWatcher>
#include <QDoubleSpinBox>


#include <QObject>
//#include "lp_import_openmesh.h"
class QOpenGLBuffer;
class QComboBox;
class QLabel;
class QSlider;
class QOpenGLShaderProgram;
class LP_ObjectImpl;


/*Define the name, version of the plugin
*/
#define LP_Plugin_Singa_Knitting_iid "cpii.rp5.SmartFashion.LP_Plugin_Singa_Knitting/0.1"


class LP_PLUGIN_SINGA_KNITTING_EXPORT LP_Plugin_Singa_Knitting: public LP_ActionPlugin
{
    Q_OBJECT
    Q_PLUGIN_METADATA(IID LP_Plugin_Singa_Knitting_iid)
    Q_INTERFACES(LP_ActionPlugin)
public:
    virtual ~LP_Plugin_Singa_Knitting();

    // LP_Functional interface
public:
    QWidget *DockUi() override;
    bool Run() override;
    bool eventFilter(QObject *watched, QEvent *event) override;
    void ResetSelection();
    bool comDistanceField();
    bool isoCurveGeneration();
    bool courseGeneration();
    bool knittingMapgeneration_new();

public slots:
    void FunctionalRender_L(QOpenGLContext *ctx, QSurface *surf, QOpenGLFramebufferObject *fbo, const LP_RendererCam &cam, const QVariant &options) override;
    void FunctionalRender_R(QOpenGLContext *ctx, QSurface *surf, QOpenGLFramebufferObject *fbo, const LP_RendererCam &cam, const QVariant &options) override;
    void PainterDraw(QWidget *glW) override;
    // LP_ActionPlugin interface
public:
    QString MenuName() override;
    QAction *Trigger() override;

private:
    QThreadPool mPool;
    bool test = false;
    bool fieldMode = false;
    bool selectMode = false;
    bool mInitialized= false;
    std::shared_ptr<QWidget> mWidget;
    QDoubleSpinBox *mDis;
    QDoubleSpinBox *mDis_course;
    QImage mImage;
    QLabel *mLabel = nullptr;
    QOpenGLShaderProgram *mProgram = nullptr;
    std::weak_ptr<LP_ObjectImpl> mObject;
    std::weak_ptr<LP_RendererCamImpl> mCam;
    QMap<uint,uint> mPoints;
    std::vector<double> Points;
    std::vector<unsigned> Faces;
    std::vector<float> point_distance;
    float maxDis;
    std::vector<float> field_color;
    std::vector<float> ocolor;
    QLabel *labelMaxDis;
    std::vector<QMap<uint,uint>> oriEdgeSet;
    std::vector<std::vector<QMap<uint,uint>>> isoEdgeSet;
    std::vector<std::vector<std::vector<float>>>oriEdgePoint;
    std::vector<std::vector<float>>oriEdgeRatio;
    std::vector<std::vector<QMap<uint,uint>>>isoNodeSequence;
    std::vector<std::vector<std::vector<float>>> firstNormal;
    std::vector<std::vector<std::vector<std::vector<float>>>>isoCurveNode;
    std::vector<std::vector<std::vector<std::vector<float>>>>isoCurveNode_resampled;
    std::vector<std::vector<std::vector<std::vector<std::vector<int>>>>> relatedNode;
    std::vector<std::vector<std::vector<std::vector<std::vector<int>>>>> relatedNode_assist;    
    std::vector<std::vector<int>> knittingMapArray;
    bool isoCurveMode = false;
    bool knittingMapMode = false;
    QOpenGLBuffer *mVBO;
    QOpenGLBuffer *mIndices;
    QSpinBox *mR, *mG, *mB;
    QSlider *mSlider;
    void initializeGL();
};

#endif // LP_PLUGIN_SINGA_KNITTING_H
