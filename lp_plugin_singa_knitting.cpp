#include "lp_plugin_singa_knitting.h"
#include "lp_renderercam.h"
//#include "lp_import_openmesh.h"
#include "Commands/lp_commandmanager.h"
#include "Commands/lp_cmd_import_opmesh.h"
//#include "lp_pick_feature_points.h"
#include "renderer/lp_glselector.h"
#include "renderer/lp_glrenderer.h"
#include <fstream>
#include <QMessageBox>
#include <QGroupBox>
#include <QMouseEvent>
#include <QOpenGLFramebufferObject>
#include <QOpenGLContext>
#include <QOpenGLShaderProgram>
#include <QOpenGLExtraFunctions>
#include <QOpenGLTexture>
#include <QOpenGLBuffer>
#include <QVBoxLayout>
#include <QLabel>
#include <QComboBox>
#include <QMatrix4x4>
#include <QPushButton>
#include <QFileDialog>
#include <QSlider>
#include <QCheckBox>
#include <QPainter>
#include <QPainterPath>
#include <QtConcurrent/QtConcurrent>



LP_Plugin_Singa_Knitting::~LP_Plugin_Singa_Knitting()
{
    emit glContextRequest([this](){
        delete mProgram;
        mProgram = nullptr;
    });
    Q_ASSERT(!mProgram);
}

bool LP_Plugin_Singa_Knitting::Run()
{
    emit glUpdateRequest();
    g_GLSelector->ClearSelected();
    return false;
}

QWidget *LP_Plugin_Singa_Knitting::DockUi()
{

    mWidget = std::make_shared<QWidget>();

    QVBoxLayout *layout = new QVBoxLayout(mWidget.get());
    layout->setContentsMargins(2,2,2,2);
    QPushButton *buttonSelect = new QPushButton(tr("Select Points"),mWidget.get());
    QPushButton *buttonReset = new QPushButton(tr("Reset"),mWidget.get());
    QGroupBox *groupA = new QGroupBox;
    QGridLayout *glayoutA = new QGridLayout;
    glayoutA->addWidget(buttonSelect,0,0);
    glayoutA->addWidget(buttonReset,0,1);
    glayoutA->setContentsMargins(5,2,5,2);
    groupA->setLayout(glayoutA);
    layout->addWidget(groupA);
    QPushButton *buttonDistance = new QPushButton(tr("Distance Filed Compute"),mWidget.get());
    labelMaxDis = new QLabel(tr("NA"),mWidget.get());
    labelMaxDis->setAlignment(Qt::AlignHCenter|Qt::AlignVCenter);
    QGroupBox *groupB = new QGroupBox;
    QGridLayout *glayoutB = new QGridLayout;
    glayoutB->addWidget(buttonDistance,0,0);
    glayoutB->addWidget(labelMaxDis,0,1);
    glayoutB->setContentsMargins(5,2,5,2);
    groupB->setLayout(glayoutB);
    layout->addWidget(groupB);
    QGroupBox *group = new QGroupBox(mWidget.get());

    mDis = new QDoubleSpinBox(mWidget.get());
    mDis->setSingleStep(0.01);
    mDis->setMaximumWidth(60);
    mDis->setValue(1);
    QPushButton *buttonIsoCurve = new QPushButton(tr("Iso-Curve Generation"),mWidget.get());
    buttonIsoCurve->setMaximumWidth(200);
    buttonIsoCurve->setContentsMargins(2,0,2,0);
    QGridLayout *glayout = new QGridLayout;
    glayout->setContentsMargins(5,2,5,2);
    glayout->addWidget(mDis, 0, 0);
    glayout->addWidget(buttonIsoCurve, 0, 1);
    group->setLayout(glayout);
    group->setMaximumHeight(100);
    layout->addWidget(group);

    QGroupBox *group_course = new QGroupBox(mWidget.get());
    mDis_course = new QDoubleSpinBox(mWidget.get());
    mDis_course->setSingleStep(0.01);
    mDis_course->setMaximumWidth(60);
    QPushButton *buttonCourse = new QPushButton(tr("Courses Generation"),mWidget.get());
    buttonCourse->setMaximumWidth(200);
    buttonCourse->setContentsMargins(2,0,2,0);
    QGridLayout *glayout_course = new QGridLayout;
    glayout_course->setContentsMargins(5,2,5,2);
    glayout_course->addWidget(mDis_course, 0, 0);
    glayout_course->addWidget(buttonCourse, 0, 1);
    group_course->setLayout(glayout_course);
    group_course->setMaximumHeight(100);
    layout->addWidget(group_course);


    QPushButton *buttonKnittingMap = new QPushButton(tr("KnittingMap Generation"),mWidget.get());
    layout->addWidget(buttonKnittingMap);

    QPushButton *button = new QPushButton(tr("Export"),mWidget.get());
    layout->addWidget(button);
    layout->addStretch();
    mWidget->setLayout(layout);

    connect(buttonSelect, &QPushButton::clicked,[this](){
        selectMode = true;
        });
    connect(buttonReset,&QPushButton::clicked,[this](){
        field_color.clear();
        mPoints.clear();
        maxDis = 0;
        Points.clear();
        Faces.clear();
        point_distance.clear();
        ocolor.clear();
        //mObject.reset();
        emit glUpdateRequest();
        //emit glUpdateRequest();
    });
    connect(buttonDistance, &QPushButton::clicked,[this](){
        selectMode = false;
        ocolor.clear();
        comDistanceField();
        emit glUpdateRequest();
        });
    connect(buttonIsoCurve, &QPushButton::clicked,[this](){
        isoCurveGeneration();
        mDis_course->setValue(2);
        mDis->setEnabled(false);
        emit glUpdateRequest();
        });
    connect(buttonCourse, &QPushButton::clicked,[this](){
        courseGeneration();
        emit glUpdateRequest();
        });
    connect(buttonKnittingMap, &QPushButton::clicked,[this](){
        knittingMapgeneration_new();
        emit glUpdateRequest();
    });



    return mWidget.get();
}

bool LP_Plugin_Singa_Knitting::comDistanceField()
{
    std::vector<uint> pointsList;
    if(mPoints.empty())
    {
        fieldMode = false;
        return false;
    }
    else{
        for(auto p =mPoints.begin();p!=mPoints.end();++p)
        {
            pointsList.push_back(p.key());
        }
    }

    static auto _isMesh = [](LP_Objectw obj){
        if ( obj.expired()){
            return LP_OpenMeshw();
        }
        return LP_OpenMeshw() = std::static_pointer_cast<LP_OpenMeshImpl>(obj.lock());
    };
    if(!mObject.lock()) return false;



    auto c = _isMesh(mObject).lock();

    auto sf_mesh = c->Mesh();
    auto sf_pt = sf_mesh->points();

    Faces.resize(sf_mesh->n_faces()*3); // Triangular mesh
    auto i_it = Faces.begin();
    for ( const auto &f : sf_mesh->faces()){
        for ( const auto &v : f.vertices()){
            //Faces.emplace_back(v.idx());
            (*i_it++) = v.idx();
        }
    }


   for( int p = 0; p < (int)sf_mesh->n_vertices(); p++){
        auto pp = sf_pt[p];
        Points.push_back(pp[0]);
        Points.push_back(pp[1]);
        Points.push_back(pp[2]);
    }

    geodesic::Mesh mesh;
    mesh.initialize_mesh_data(Points, Faces); // create internal mesh data structure including edges
    geodesic::GeodesicAlgorithmExact algorithm(&mesh); // create exact algorithm for the mesh
    std::vector<geodesic::SurfacePoint> path;
    for(const auto p:pointsList)
    {
        geodesic::SurfacePoint source(&mesh.vertices()[p]);
        path.push_back(source);
    }

    // Geodesic Distance Field
    algorithm.propagate(path); // cover the whole mesh
    for (unsigned i = 0; i < mesh.vertices().size(); ++i) {
        geodesic::SurfacePoint p(&mesh.vertices()[i]);
        double distance;
        algorithm.best_source(p, distance); // for a given surface point, find closets source and distance to this source
        point_distance.emplace_back(distance);
        if(i == 0){
          maxDis = distance;
        }
        else if(distance > maxDis){
          maxDis = distance;
        }
    }
    labelMaxDis->setText(QString::number(maxDis));
    for(int i=0; i< (int)sf_mesh->n_vertices(); i++){
        float scale = point_distance[i]/maxDis;
        if(scale<0.5f){
          field_color.push_back(0.0f);
          field_color.push_back(1.0f-scale*2.0f);
          field_color.push_back(scale*2.0f);
        }
        if(scale==0.5f || scale>0.5f){
          field_color.push_back(scale*2.0f-1.0f);
          field_color.push_back(0.0f);
          field_color.push_back(1.0f/scale-1.0f);
        }
    }
    emit glUpdateRequest();

    return true;
}

bool LP_Plugin_Singa_Knitting::isoCurveGeneration()
{

    if(maxDis==0) return false;
    double isoLength = mDis->value();
    ocolor.clear();
    field_color.clear();
    //fieldMode = false;
    isoCurveMode = true;
    double isoValue;
    isoValue= isoLength/maxDis;
    int isoNum = 1/isoValue+0.5;
    static auto _isMesh = [](LP_Objectw obj){
        if ( obj.expired()){
            return LP_OpenMeshw();
        }
        return LP_OpenMeshw() = std::static_pointer_cast<LP_OpenMeshImpl>(obj.lock());
    };
    if(!mObject.lock()) return false;
    auto c = _isMesh(mObject).lock();
    auto sf_mesh = c->Mesh();
    for(int i =0;i<isoNum;i++)
    {
        double length = (i+0.5)*isoLength;
        std::vector<std::vector<float>> oriEdgeList;
        std::vector<float> oriRationList;
        std::vector<QMap<uint,uint>> singleEdge;
        QMap<uint,uint> edgeSet;
        for(int j = 0;j<sf_mesh->n_vertices();j++)
        {
            if (point_distance[j]==length)
                point_distance[j]+=0.0001;
        }

        for (auto e_it=sf_mesh->edges_begin();e_it!=sf_mesh->edges_end() ; ++e_it)
        {
            if((point_distance[e_it->vertex(0).idx()]>length && point_distance[e_it->vertex(1).idx()]<length)||(point_distance[e_it->vertex(0).idx()]<length && point_distance[e_it->vertex(1).idx()]>length))
            {
                std::vector<float> edge_point;
                edge_point.emplace_back(sf_mesh->point(sf_mesh->vertex_handle(e_it->vertex(0).idx())).data()[0]);
                edge_point.emplace_back(sf_mesh->point(sf_mesh->vertex_handle(e_it->vertex(0).idx())).data()[1]);
                edge_point.emplace_back(sf_mesh->point(sf_mesh->vertex_handle(e_it->vertex(0).idx())).data()[2]);
                edge_point.emplace_back(sf_mesh->point(sf_mesh->vertex_handle(e_it->vertex(1).idx())).data()[0]);
                edge_point.emplace_back(sf_mesh->point(sf_mesh->vertex_handle(e_it->vertex(1).idx())).data()[1]);
                edge_point.emplace_back(sf_mesh->point(sf_mesh->vertex_handle(e_it->vertex(1).idx())).data()[2]);
                float ratio = (length-point_distance[e_it->vertex(0).idx()])/(point_distance[e_it->vertex(1).idx()]-point_distance[e_it->vertex(0).idx()]);
                oriRationList.push_back(ratio);
                oriEdgeList.push_back(edge_point);
                edgeSet.insert(e_it->idx(),edgeSet.size());
            }
        }
        oriEdgeRatio.push_back(oriRationList);
        oriEdgePoint.push_back(oriEdgeList);
        oriEdgeSet.push_back(edgeSet);

        //Find the side of curves
        //bool isClosed = true;
        for(auto edge_key = edgeSet.begin();edge_key!=edgeSet.end();++edge_key)
        {
            auto edge = edge_key.key();
            if(sf_mesh->is_boundary(sf_mesh->edge_handle(edge)))
            {
                bool isRepeat = false;
                for(int j = 0;j<singleEdge.size();j++)
                {
                    if(singleEdge[j].contains(edge))
                    {
                        isRepeat = true;
                        break;
                    }
                }
                if(!isRepeat)
                {
                    QMap<uint,uint> newEdge;
                    newEdge.insert(edge,newEdge.count());
                    bool isEnd = false;
                    auto edge_tem = edge;
                    for(;!isEnd;)
                    {
                        isEnd = true;
                        for(OpenMesh::TriConnectivity::EdgeIter e_it = sf_mesh->edges_begin(); e_it!=sf_mesh->edges_end(); ++e_it)
                        {
                            if(e_it->idx()==edge_tem)
                            {
                                bool isFind = false;
                                if(e_it->halfedge(0).is_valid()&&e_it->halfedge(0).face().is_valid())
                                {
                                    for(auto e_iter_tem:e_it->halfedge(0).face().edges())
                                    {
                                        if(newEdge.contains(e_iter_tem.idx())) continue;
                                        else
                                        {
                                            if(edgeSet.contains(e_iter_tem.idx()))
                                            {
                                                newEdge.insert(e_iter_tem.idx(),newEdge.count());
                                                edge_tem = e_iter_tem.idx();
                                                isFind = true;
                                                isEnd = false;
                                                break;
                                            }
                                        }
                                    }
                                }
                                if(!isFind && e_it->halfedge(1).is_valid()&&e_it->halfedge(1).face().is_valid())
                                {
                                    for(auto e_iter_tem:e_it->halfedge(1).face().edges())
                                    {
                                        if(newEdge.contains(e_iter_tem.idx())) continue;
                                        else
                                        {
                                            if(edgeSet.contains(e_iter_tem.idx()))
                                            {
                                                newEdge.insert(e_iter_tem.idx(),newEdge.count());
                                                edge_tem = e_iter_tem.idx();
                                                isEnd = false;
                                                break;
                                            }
                                        }
                                    }
                                }
                                break;
                            }
                        }
                    }
                    singleEdge.push_back(newEdge);
                    if(i==0 && singleEdge.size()>1) {qDebug()<<"The first isoCurve contains more than 1 splited curves";return false;}
                    //isClosed = false;
                }
            }
        }

        int count = 0;
        for(int j = 0;j<singleEdge.size();j++)
        {
            count+=singleEdge[j].count();
        }
        //qDebug()<<"The number of No. "<< i <<" connected curves is: "<<singleEdge.size();
        isoNodeSequence.push_back(singleEdge);

    }
    //Calculate the isoNode
    for(int i =0;i<isoNodeSequence.size();i++)
    {
        std::vector<std::vector<std::vector<float>>>isoCurveNode_1;
        std::vector<std::vector<float>> firstNormal_1;
        for(int j = 0;j<isoNodeSequence[i].size();j++)
        {
            std::vector<std::vector<float>>isoCurveNode_2;
            std::vector<float> firstNormal_2;
            for(int k = 0;k<isoNodeSequence[i][j].size();k++)
            {
                for(auto kk = oriEdgeSet[i].begin();kk!=oriEdgeSet[i].end();++kk)
                {
                    if(kk.key()==isoNodeSequence[i][j].key(k))
                    {
                        if(k == 0)
                        {
                            auto normal0 = sf_mesh->calc_normal(sf_mesh->edge_handle(isoNodeSequence[i][j].key(0)));
                            for(int l =0;l<3;l++)firstNormal_2.emplace_back(normal0.data()[l]);
                        }
                        std::vector<float>isoCurveNode_3;
                        float ratio = oriEdgeRatio[i][kk.value()];
                        std::vector<float> edgePoint;
                        for(int l = 0; l < 6;l++) edgePoint.emplace_back(oriEdgePoint[i][kk.value()][l]);
                        for(int l = 0; l < 3;l++)
                            //isoCurveNode_3.emplace_back(edgePoint[l]);
                            isoCurveNode_3.emplace_back((1.0-ratio)*edgePoint[l]+ratio*edgePoint[l+3]);
                        //qDebug()<<edgePoint<<"\n"<<ratio<<"\n"<<isoCurveNode_3;;
                        //qDebug()<<ratio<<edgePoint<<isoCurveNode_3;
                        isoCurveNode_2.push_back(isoCurveNode_3);
                        break;
                    }
                }
            }
            firstNormal_1.emplace_back(firstNormal_2);
            isoCurveNode_1.emplace_back(isoCurveNode_2);
        }
        firstNormal.emplace_back(firstNormal_1);
        isoCurveNode.emplace_back(isoCurveNode_1);
    }
//    return false;

    //smooth the curves and delete the short curves(Nodenum<=3)
    for(int i =0;i<isoCurveNode.size();i++)
    {
        for(int j = 0;j<isoCurveNode[i].size();j++)
        {
            std::vector<std::vector<float>> curveTem;
            if(isoCurveNode[i][j].size()<3) continue;
            curveTem.emplace_back(isoCurveNode[i][j][0]);
            for(int k = 1;k<isoCurveNode[i][j].size()-1;k++)
            {
                std::vector<float> v0,v1;
                float dot = 0;
                for(int l = 0;l<3;l++)
                {
                    v0.emplace_back(isoCurveNode[i][j][k][l]-isoCurveNode[i][j][k-1][l]);
                    v1.emplace_back(isoCurveNode[i][j][k+1][l]-isoCurveNode[i][j][k][l]);
                    dot+=v0[l]*v1[l];
                }
                if(dot>=0) curveTem.emplace_back(isoCurveNode[i][j][k]);
                //else qDebug()<<"Removed";
            }
            curveTem.emplace_back(isoCurveNode[i][j][isoCurveNode[i][j].size()-1]);
            isoCurveNode[i][j].clear();
            for(int k = 0;k<curveTem.size();k++)
            {
                isoCurveNode[i][j].emplace_back(curveTem[k]);
            }
        }
    }
    for(int i =0;i<isoCurveNode.size();i++)
    {
        for(int j = 0;j<isoCurveNode[i].size();j++)
        {
            if(isoCurveNode[i].size()==1) continue;
            if(isoCurveNode[i][j].size()<=3)
            {
                isoCurveNode[i].erase(isoCurveNode[i].begin()+j);
            }
        }
    }

    //redirect the curves
    if(isoCurveNode.size()<=1){qDebug()<<"The number of IsoCurve is less than 2";return false;}
    std::vector<float>crossDot;
    for(int i = 0;i<isoCurveNode.size();i++)
    {
        for(int j = 0;j<isoCurveNode[i].size();j++)
        {
            auto referNormal = firstNormal[i][j];
            std::vector<float>n0 = isoCurveNode[i][j][0];
            std::vector<float>n1 = isoCurveNode[i][j][1];
            std::vector<float>v0;
            std::vector<float>v1;
            std::vector<float>n2;
            float minDistance = 999999.9;
            for(int k = 0;k<3;k++)
                 v0.emplace_back(n1[k]-n0[k]);
            //find the cloest point at second curve
            if(i!=isoCurveNode.size()-1)
            {
                for(int k = 0;k<isoCurveNode[i+1].size();k++)
                {
                    for(int l = 0;l<isoCurveNode[i+1][k].size();l++)
                    {
                        std::vector<float> nextNode;
                        for(int m = 0;m<3;m++)
                            nextNode.emplace_back(isoCurveNode[i+1][k][l][m]);
                        float distance = 0;
                        for(int m = 0;m<3;m++)
                        {
                            distance+=pow((nextNode[m]-n0[m]),2);
                        }
                        if(distance<minDistance)
                        {
                            n2.clear();
                            minDistance = distance;
                            for(int m = 0;m<3;m++)
                                n2.emplace_back(nextNode[m]);
                        }
                    }
                }
                for(int k = 0;k<3;k++)
                     v1.emplace_back(n0[k]-n2[k]);
            }
            if(minDistance>2*isoLength||i==isoCurveNode.size()-1)
            {
                minDistance=99999.9;
                for(int k = 0;k<isoCurveNode[i-1].size();k++)
                {
                    for(int l = 0;l<isoCurveNode[i-1][k].size();l++)
                    {
                        std::vector<float> nextNode;
                        for(int m = 0;m<3;m++)
                            nextNode.emplace_back(isoCurveNode[i-1][k][l][m]);
                        float distance = 0;
                        for(int m = 0;m<3;m++)
                        {
                            distance+=pow((nextNode[m]-n0[m]),2);
                        }
                        if(distance<minDistance)
                        {
                            n2.clear();
                            minDistance = distance;
                            for(int m = 0;m<3;m++)
                                n2.emplace_back(nextNode[m]);

                        }
                    }
                }
                //qDebug()<<"1321"<<n2;
                v1.clear();
                //qDebug()<<"1321"<<n2;
                for(int k = 0;k<3;k++)
                     v1.emplace_back(n2.data()[k]-n0.data()[k]);

            }

            std::vector<float>thisCrossDot;
            thisCrossDot.emplace_back(v0[1]*v1[2]-v0[2]*v1[1]);
            thisCrossDot.emplace_back(v0[2]*v1[0]-v0[0]*v1[2]);
            thisCrossDot.emplace_back(v0[0]*v1[1]-v0[1]*v1[0]);
            float result=0;
            for(int k = 0;k<3;k++)
            result+=thisCrossDot[k]*referNormal[k];
            if(result<0)
                std::reverse(isoCurveNode[i][j].begin(),isoCurveNode[i][j].end());
        }
    }
    //unify the curves
    //step1: detect the first isoCurve which is conducted at the begining
    //step2: sort the rest isoCurves
    std::vector<std::vector<std::vector<int>>> nearestNodeIdx;
    for(int i = 0;i<isoCurveNode.size();i++)
    {
        if(i ==0 || isoCurveNode[i].size()==1)
        {
            std::vector<std::vector<int>> newIdx;
            std::vector<int> idx;
            idx.emplace_back(0);
            idx.emplace_back(0);
            newIdx.push_back(idx);
            nearestNodeIdx.emplace_back(newIdx);
        }
        else
        {
            std::vector<std::vector<int>> newIdx;
            for(int j = 0;j<isoCurveNode[i].size();j++)
            {
                std::vector<float>n0 = isoCurveNode[i][j][0];
                std::vector<int> idx;
                float minDistance = 9999.9;
                for(int k = 0;k<isoCurveNode[i-1].size();k++)
                {
                    for(int l = 0;l<isoCurveNode[i-1][k].size();l++)
                    {
                        std::vector<float> nextNode;
                        for(int m = 0;m<3;m++)
                            nextNode.emplace_back(isoCurveNode[i-1][k][l][m]);
                        float distance = 0;
                        for(int m = 0;m<3;m++)
                        {
                            distance+=pow((nextNode[m]-n0[m]),2);
                        }
                        if(distance<minDistance)
                        {
                            idx.clear();
                            minDistance = distance;
                            idx.emplace_back(k);
                            idx.emplace_back(l);
                        }
                    }
                }
                newIdx.emplace_back(idx);
            }
            nearestNodeIdx.emplace_back(newIdx);
        }
        //qDebug()<<"Before:"<<nearestNodeIdx[i];
        if(nearestNodeIdx[i].size()==1) continue;
        for(int j=0;j<nearestNodeIdx[i].size()-1;j++)
        {
            for(int k = 0;k<nearestNodeIdx[i].size()-1-j;k++)
            {
                if(nearestNodeIdx[i][k][0]>nearestNodeIdx[i][k+1][0])
                {
                    std::swap(nearestNodeIdx[i][k],nearestNodeIdx[i][k+1]);
                    std::swap(isoCurveNode[i][k],isoCurveNode[i][k+1]);
                }
            }
        }
        for(int j=0;j<nearestNodeIdx[i].size()-1;j++)
        {
            for(int k = 0;k<nearestNodeIdx[i].size()-1-j;k++)
            {
                if(nearestNodeIdx[i][k][0]==nearestNodeIdx[i][k+1][0] && nearestNodeIdx[i][k][1]>nearestNodeIdx[i][k+1][1])
                {
                    std::swap(nearestNodeIdx[i][k],nearestNodeIdx[i][k+1]);
                    std::swap(isoCurveNode[i][k],isoCurveNode[i][k+1]);
                }
            }
        }
        //qDebug()<<"After:"<<nearestNodeIdx[i];
    }
    return true;
}

bool LP_Plugin_Singa_Knitting::courseGeneration()
{
    float gap = mDis_course->value()*mDis->value();
    std::vector<std::vector<float>> curveLength;

    for(int i =0;i<isoCurveNode.size();i++)
    {
        std::vector<float> curveLength_1;
        for(int j =0;j<isoCurveNode[i].size();j++)
        {
            float curveLength_2 = 0;
            for(int k = 0;k<isoCurveNode[i][j].size()-1;k++)
            {
                std::vector<float> v;
                for(int l = 0;l<3;l++)
                {
                    v.emplace_back(isoCurveNode[i][j][k+1][l]-isoCurveNode[i][j][k][l]);
                }
                curveLength_2+=sqrt(pow(v[0],2)+pow(v[1],2)+pow(v[2],2));
            }
            //if(curveLength_2<0.5*gap) isoCurveNode[i].erase(isoCurveNode[i].begin()+j);
            curveLength_1.emplace_back(curveLength_2);
        }
        //if(isoCurveNode[i].size()==0)isoCurveNode.erase(isoCurveNode.begin()+i);
        curveLength.emplace_back(curveLength_1);
    }

    relatedNode.resize(curveLength.size());
    isoCurveNode_resampled.resize(curveLength.size());
    relatedNode_assist.resize(curveLength.size());
    for(int i =0;i<curveLength.size();i++)
    {
        relatedNode[i].resize(curveLength[i].size());
        isoCurveNode_resampled[i].resize(curveLength[i].size());
        relatedNode_assist[i].resize(curveLength[i].size());
        //qDebug()<<curveLength[i].size()<<relatedNode[i].size()<<isoCurveNode_resampled[i].size();
        for(int j =0;j<curveLength[i].size();j++)
        {
            if(curveLength[i][j]<gap) relatedNode[i][j].resize(2);
            else relatedNode[i][j].resize(round(curveLength[i][j]/gap)+1);
            relatedNode_assist[i][j].resize(relatedNode[i][j].size());
            //qDebug()<<isoCurveNode[i][j].size()<<curveLength[i][j]<<int(curveLength[i][j]/gap)+1<<relatedNode[i][j].size();
            isoCurveNode_resampled[i][j].emplace_back(isoCurveNode[i][j][0]);
            for(int k =1; k<relatedNode[i][j].size()-1; k++)
            {
                float new_gap = k * curveLength[i][j]/(relatedNode[i][j].size()-1);
                float distance = 0;
                for(int l = 0;l < isoCurveNode[i][j].size()-1;l++)
                {
                    distance+=sqrt(pow(isoCurveNode[i][j][l+1][0]-isoCurveNode[i][j][l][0],2)+pow(isoCurveNode[i][j][l+1][1]-isoCurveNode[i][j][l][1],2)+pow(isoCurveNode[i][j][l+1][2]-isoCurveNode[i][j][l][2],2));
                    if(distance>new_gap)
                    {
                        float prevDis = distance-sqrt(pow(isoCurveNode[i][j][l+1][0]-isoCurveNode[i][j][l][0],2)+pow(isoCurveNode[i][j][l+1][1]-isoCurveNode[i][j][l][1],2)+pow(isoCurveNode[i][j][l+1][2]-isoCurveNode[i][j][l][2],2));
                        float ratio = (new_gap-prevDis)/(distance-prevDis);
                        std::vector<float> newNode;
                        for(int m = 0;m<3;m++)
                            newNode.emplace_back((1-ratio)*isoCurveNode[i][j][l][m]+ratio*isoCurveNode[i][j][l+1][m]);
                        isoCurveNode_resampled[i][j].emplace_back(newNode);
                        break;
                    }
                }
            }
            isoCurveNode_resampled[i][j].emplace_back(isoCurveNode[i][j][isoCurveNode[i][j].size()-1]);
            //qDebug()<<isoCurveNode[i][j].size()<<curveLength[i][j]<<isoCurveNode_resampled[i][j].size()<<relatedNode[i][j].size();
        }
    }

    //connect courses

    for(int i =0; i<isoCurveNode_resampled.size();i++)
    {
        for(int j = 0;j<isoCurveNode_resampled[i].size();j++)
        {
            if(i!=0)
            {
                //connect down
                for(int k = 0;k<isoCurveNode_resampled[i][j].size();k++)
                {
                    if(relatedNode_assist[i][j][k].size()>0) continue;
                    std::vector<float>n0 = isoCurveNode_resampled[i][j][k];
                    std::vector<int> idx;
                    std::vector<int> idx_self;
                    idx_self.emplace_back(j);
                    idx_self.emplace_back(k);
                    float minDistance = 3*gap;
                    for(int jj = 0;jj<isoCurveNode_resampled[i-1].size();jj++)
                    {
                        for(int kk = 0;kk<isoCurveNode_resampled[i-1][jj].size();kk++)
                        {
                            std::vector<float> nextNode;
                            for(int m = 0;m<3;m++)
                                nextNode.emplace_back(isoCurveNode_resampled[i-1][jj][kk][m]);
                            float distance = 0;
                            for(int m = 0;m<3;m++)
                            {
                                distance+=pow((nextNode[m]-n0[m]),2);
                            }
                            bool isIntersection = false;
                            for(int jjj = 0;jjj<jj;jjj++)
                            {
                                for(int kkk = 0;kkk<kk;kkk++)
                                {
                                    for(int l = 0;l<relatedNode_assist[i][jjj][kkk].size();l++)
                                        if(relatedNode_assist[i][jjj][kkk][l][0]>jj||relatedNode_assist[i][jjj][kkk][l][2]>kk)
                                        {
                                            isIntersection = true;
                                            break;
                                        }
                                }
                            }
                            if(distance<minDistance && !isIntersection)
                            {
                                idx.clear();
                                minDistance = distance;
                                idx.emplace_back(jj);
                                idx.emplace_back(kk);
                            }
                        }
                    }
                    relatedNode[i-1][idx[0]][idx[1]].emplace_back(idx_self);
                    relatedNode_assist[i][j][k].emplace_back(idx);
                }
            }
            if(i!=isoCurveNode_resampled.size()-1)
            {
                //connect up
                for(int k = 0;k<isoCurveNode_resampled[i][j].size();k++)
                {
                    std::vector<float>n0 = isoCurveNode_resampled[i][j][k];
                    std::vector<int> idx;
                    std::vector<int> idx_self;
                    idx_self.emplace_back(j);
                    idx_self.emplace_back(k);
                    float minDistance = 3*gap;
                    for(int jj = 0;jj<isoCurveNode_resampled[i+1].size();jj++)
                    {
                        for(int kk = 0;kk<isoCurveNode_resampled[i+1][jj].size();kk++)
                        {
                            std::vector<float> nextNode;
                            for(int m = 0;m<3;m++)
                                nextNode.emplace_back(isoCurveNode_resampled[i+1][jj][kk][m]);
                            float distance = 0;
                            for(int m = 0;m<3;m++)
                            {
                                distance+=pow((nextNode[m]-n0[m]),2);
                            }
                            if(distance<minDistance)
                            {
                                idx.clear();
                                minDistance = distance;
                                idx.emplace_back(jj);
                                idx.emplace_back(kk);
                            }
                        }
                    }
                    if(!idx.empty())
                    {
                        relatedNode[i][j][k].emplace_back(idx);
                        relatedNode_assist[i+1][idx[0]][idx[1]].emplace_back(idx_self);
                    }
                }
            }
        }
    }

    return true;
}

bool LP_Plugin_Singa_Knitting::knittingMapGeneration()
{
    knittingMapMode = true;
    std::vector<std::vector<int>> knittingMapLeft;
    std::vector<std::vector<int>> knittingMapRight;
    std::vector<std::vector<int>> nodeLength;
    std::vector<std::vector<std::vector<int>>> relatedNodeUp_new;
    std::vector<std::vector<std::vector<int>>> relatedNodeDown_new;
    std::vector<std::vector<int>> result;
    //re-assign the relatedNode
    for(int i = 0;i<relatedNode.size();i++)
    {
        std::vector<std::vector<int>> rNode;
        for(int j = 0;j<relatedNode[i].size();j++)
        {
            for(int k = 0;k<relatedNode[i][j].size();k++)
            {
                std::vector<int> rNode_1;
                for(int l = 0;l<relatedNode[i][j][k].size();l++)
                {
                    if(relatedNode[i][j][k][l][0]!=0&&i!=relatedNode.size()-1)
                    {
                        rNode_1.emplace_back(relatedNode[i][j][k][l][1]+relatedNode[i+1][relatedNode[i][j][k][l][0]-1].size());
                    }
                    else if(relatedNode[i][j][k][l][0]!=0&&i==relatedNode.size()-1)
                    {
                        rNode_1.emplace_back(relatedNode[i][j][k][l][1]+relatedNode_assist[i][relatedNode[i][j][k][l][0]-1].size());
                    }
                    else
                        rNode_1.emplace_back(relatedNode[i][j][k][l][1]);
                }
                rNode.emplace_back(rNode_1);
            }
        }
        relatedNodeUp_new.emplace_back(rNode);
    }
    for(int i = 0;i<relatedNode_assist.size();i++)
    {
        std::vector<std::vector<int>> rNode;
        for(int j = 0;j<relatedNode_assist[i].size();j++)
        {
            for(int k = 0;k<relatedNode_assist[i][j].size();k++)
            {
                std::vector<int> rNode_1;
                for(int l = 0;l<relatedNode_assist[i][j][k].size();l++)
                {
                    if(relatedNode_assist[i][j][k][l][0]!=0 && i!=0)
                        rNode_1.emplace_back(relatedNode_assist[i][j][k][l][1]+relatedNode_assist[i-1][relatedNode_assist[i][j][k][l][0]-1].size());
                    else if(relatedNode_assist[i][j][k][l][0]!=0&&i==0)
                    {
                        rNode_1.emplace_back(relatedNode_assist[i][j][k][l][1]+relatedNode[i][relatedNode[i][j][k][l][0]-1].size());
                    }
                    else
                        rNode_1.emplace_back(relatedNode_assist[i][j][k][l][1]);
                }
                rNode.emplace_back(rNode_1);
            }
        }
        relatedNodeDown_new.emplace_back(rNode);
    }
    //Delete isolated Node
    for(int i =0;i<relatedNodeUp_new.size();i++)
    {
        for(int j = 0;j<relatedNodeUp_new[i].size();j++)
        {
            if(relatedNodeUp_new[i][j].size()==0&&relatedNodeDown_new[i][j].size()==0)
            {
                qDebug()<<"There is an isolated Node--1";
                relatedNodeDown_new[i].erase(relatedNodeDown_new[i].begin()+j);
                relatedNodeUp_new[i].erase(relatedNodeUp_new[i].begin()+j);
            }
        }
    }
    //sort the Node
    for(int i =0;i<relatedNodeUp_new.size();i++)
    {
        for(int j = 0;j<relatedNodeUp_new[i].size();j++)
        {
            if(relatedNodeUp_new[i][j].size()<=1) continue;
            int a, b;
            for (a = 0; a < relatedNodeUp_new[i][j].size() - 1; a++)
                for (b = 0; b < relatedNodeUp_new[i][j].size() - 1 - a; b++)
                    if (relatedNodeUp_new[i][j][b] > relatedNodeUp_new[i][j][b + 1])
                        std::swap(relatedNodeUp_new[i][j][b], relatedNodeUp_new[i][j][b + 1]);
        }
        //qDebug()<<relatedNodeUp_new[i];
    }
    for(int i =0;i<relatedNodeDown_new.size();i++)
    {
        for(int j = 0;j<relatedNodeDown_new[i].size();j++)
        {
            if(relatedNodeDown_new[i][j].size()<=1) continue;
            int a, b;
            for (a = 0; a < relatedNodeDown_new[i][j].size() - 1; a++)
                for (b = 0; b < relatedNodeDown_new[i][j].size() - 1 - a; b++)
                    if (relatedNodeDown_new[i][j][b] > relatedNodeDown_new[i][j][b + 1])
                        std::swap(relatedNodeDown_new[i][j][b], relatedNodeDown_new[i][j][b + 1]);
        }
    }

    //Travel the Node(Get all branch);
    int firstLength = 0;
    for(int i = 0;i<relatedNodeUp_new.size();i++)
    {
        std::vector<int>nodeLength_1;
        for(int j = 0;j<relatedNodeUp_new[i].size();j++)
        {
            int branchNum = 0;
            std::vector<int>nextNode;
            std::vector<int>nextNode_last;
            for(int k = 0;k<relatedNodeUp_new[i][j].size();k++)
                nextNode.emplace_back(relatedNodeUp_new[i][j][k]);
            std::set<int>s(nextNode.begin(), nextNode.end());
            nextNode.assign(s.begin(), s.end());
            nextNode_last.emplace_back(j);
            for(int a =i+1;a<relatedNodeUp_new.size();a++)
            {
                std::vector<int>nextNode_new;
                nextNode_new.clear();
                for(int k = 0;k<nextNode.size();k++)
                {
                    if(relatedNodeUp_new[a][nextNode[k]].size()==0) branchNum++;
                    if(relatedNodeDown_new[a][nextNode[k]].size()>1)
                    {
                        for(int l = 0;l<relatedNodeDown_new[a][nextNode[k]].size();l++)
                        {
                            for(int m = 0;m<nextNode_last.size();m++)
                            {
                                if(nextNode_last[m]==relatedNodeDown_new[a][nextNode[k]][l]) branchNum++;
                            }
                        }
                        branchNum--;
                    }
                    for(int l = 0;l<relatedNodeUp_new[a][nextNode[k]].size();l++)
                        nextNode_new.emplace_back(relatedNodeUp_new[a][nextNode[k]][l]);
                }
                std::set<int>s1(nextNode_new.begin(), nextNode_new.end());
                nextNode_new.assign(s1.begin(), s1.end());
                nextNode_last.clear();
                nextNode_last = nextNode;
                nextNode.clear();
                nextNode=nextNode_new;
            }
            nodeLength_1.emplace_back(branchNum);
        }

        int totalLength= 0;
        if(i==0)
            for(int j = 0;j<nodeLength_1.size();j++)
            {
                if(nodeLength_1[j]==0)
                    firstLength++;
                else
                    firstLength+=nodeLength_1[j];
            }
        else
        {
            for(int j = 0;j<nodeLength_1.size();j++)
            {
                if(nodeLength_1[j]==0)
                    totalLength++;
                else
                    totalLength+=nodeLength_1[j];
            }
            if(totalLength<firstLength)
            {
                nodeLength_1.clear();
                for(int j = 0;j<relatedNodeDown_new[i].size();j++)
                {
                    int branchNum =0;
                    std::vector<int>nextNode;
                    std::vector<int>nextNode_last;
                    for(int k = 0;k<relatedNodeDown_new[i][j].size();k++)
                        nextNode.emplace_back(relatedNodeDown_new[i][j][k]);
                    std::set<int>s(nextNode.begin(), nextNode.end());
                    nextNode.assign(s.begin(), s.end());
                    nextNode_last.emplace_back(j);
                    for(int a =i-1;a>-1;a--)
                    {
                        std::vector<int>nextNode_new;
                        nextNode_new.clear();
                        for(int k = 0;k<nextNode.size();k++)
                        {
                            if(relatedNodeDown_new[a][nextNode[k]].size()==0) branchNum++;
                            if(relatedNodeUp_new[a][nextNode[k]].size()>1)
                            {
                                for(int l = 0;l<relatedNodeUp_new[a][nextNode[k]].size();l++)
                                {
                                    for(int m = 0;m<nextNode_last.size();m++)
                                    {
                                        if(nextNode_last[m]==relatedNodeUp_new[a][nextNode[k]][l]) branchNum++;
                                    }
                                }
                                branchNum--;
                            }
                            for(int l = 0;l<relatedNodeDown_new[a][nextNode[k]].size();l++)
                                nextNode_new.emplace_back(relatedNodeDown_new[a][nextNode[k]][l]);
                        }
                        std::set<int>s1(nextNode_new.begin(), nextNode_new.end());
                        nextNode_new.assign(s1.begin(), s1.end());
                        nextNode_last.clear();
                        nextNode_last = nextNode;
                        nextNode.clear();
                        nextNode=nextNode_new;
                    }
                    nodeLength_1.emplace_back(branchNum);
                }
            }
        }

        nodeLength.emplace_back(nodeLength_1);
        totalLength= 0;
        for(int j = 0;j<nodeLength_1.size();j++)
        {
            totalLength+=nodeLength_1[j];
        }
        qDebug()<<i<<nodeLength_1<<totalLength;

    }
    return false;
    //Generate knitting map
    for(int i=0;i<nodeLength.size();i++)
    {
        std::vector<int> knittingMapLeft_1;
        std::vector<int> knittingMapRight_1;
        int buff = 0;
        bool intension = false;
        for(int j = 0;j<nodeLength[i].size();j++)
        {
//-------------------------------Version 2 (June 16)-------------------------------------------------------
            if(i>0 && relatedNodeDown_new[i][j].size()>0)
            {
                for(;;)
                {
                    if(knittingMapRight[i-1][knittingMapRight_1.size()]>=*std::min_element(relatedNodeDown_new[i][j].begin(),relatedNodeDown_new[i][j].end()))
                    {
                        if(!knittingMapRight_1.empty()&&knittingMapRight_1[knittingMapRight_1.size()-1]==-1&&knittingMapLeft_1[knittingMapLeft_1.size()-1]==-1)
                        {
                            if(intension) intension = false;
                            else
                            {
                                knittingMapRight_1[knittingMapRight_1.size()-1]=-2;
                                knittingMapLeft_1[knittingMapLeft_1.size()-1]=-2;
                            }
                        }
                        break;
                    }
                    knittingMapRight_1.emplace_back(-1);
                    knittingMapLeft_1.emplace_back(-1);
                }
            }
            if(relatedNodeUp_new[i][j].size()==0&&i!=nodeLength.size()-1)
            {
                if(j-buff-1<0)
                    for(int k = 0;k<relatedNodeDown_new[i][j].size();k++)
                    {
                        knittingMapRight_1.emplace_back(-2);
                        knittingMapLeft_1.emplace_back(j);
                    }
                else
                    for(int k = 0;k<relatedNodeDown_new[i][j].size();k++)
                    {
                        knittingMapRight_1.emplace_back(-2);
                        knittingMapLeft_1.emplace_back(j);
                    }
                buff++;
                continue;
            }
            if(relatedNodeDown_new[i][j].size()==0&&i!=0)
            {
                if(j-buff-1<0)
                    for(int k = 0;k<nodeLength[i][j];k++)
                    {
                        knittingMapLeft_1.emplace_back(-2);
                        knittingMapRight_1.emplace_back(j);
                    }
                else
                    for(int k = 0;k<nodeLength[i][j];k++)
                    {
                        knittingMapLeft_1.emplace_back(-2);
                        knittingMapRight_1.emplace_back(j);
                    }
                buff++;
                continue;
            }
            if(i==nodeLength.size()-1)
            {
                for(int k = 0;k<relatedNodeDown_new[i][j].size();k++)
                {
                    knittingMapRight_1.emplace_back(-2);
                    knittingMapLeft_1.emplace_back(j);
                }
                buff++;
                continue;
            }
            int upDown = 0;
            if(relatedNodeDown_new[i][j].size()>nodeLength[i][j]&&i!=nodeLength.size()-1)
            {

                if(relatedNodeDown_new[i+1][relatedNodeUp_new[i][j][0]].size()>1)
                    for(int k =0;k<relatedNodeDown_new[i+1][relatedNodeUp_new[i][j][0]].size();k++)
                    {
                        if(relatedNodeDown_new[i+1][relatedNodeUp_new[i][j][0]][k]==j) continue;
                        if(relatedNodeDown_new[i+1][relatedNodeUp_new[i][j][0]][k]<j) upDown++;
                        else upDown--;
                    }
                else if(relatedNodeDown_new[i+1][relatedNodeUp_new[i][j][0]].size()==1)
                {
                    if(relatedNodeUp_new[i][j][0]==0) upDown--;
                    else if(relatedNodeUp_new[i][j][0]==relatedNodeDown_new[i+1].size()-1) upDown++;
                }
                if(upDown>0)
                    for(int k = 0;k<relatedNodeDown_new[i][j].size();k++)
                    {
                        knittingMapLeft_1.emplace_back(j);
                        if(j-buff-1<0)
                            if(k!=0)
                                knittingMapRight_1.emplace_back(-1);
                            else
                                knittingMapRight_1.emplace_back(j);
                        else
                            if(k!=0)
                                knittingMapRight_1.emplace_back(-1);
                            else
                                knittingMapRight_1.emplace_back(j);

                    }
                else if(upDown<0)
                    for(int k = 0;k<relatedNodeDown_new[i][j].size();k++)
                    {
                        knittingMapLeft_1.emplace_back(j);
                        if(j-buff-1<0)
                            if(k!=relatedNodeDown_new[i][j].size()-1)
                                knittingMapRight_1.emplace_back(-1);
                            else
                                knittingMapRight_1.emplace_back(j);
                        else
                            if(k!=relatedNodeDown_new[i][j].size()-1)
                                knittingMapRight_1.emplace_back(-1);
                            else
                                knittingMapRight_1.emplace_back(j);
                    }
                else
                {
                    for(int k = 0;k<relatedNodeDown_new[i][j].size();k++)
                    {
                        knittingMapLeft_1.emplace_back(j);
                        knittingMapRight_1.emplace_back(j);
                    }
                    intension = true;
                }
            }
            else
                for(int k = 0;k<nodeLength[i][j];k++)
                {
                    knittingMapRight_1.emplace_back(j);
                    knittingMapLeft_1.emplace_back(j);
                }


        }

        knittingMapRight.emplace_back(knittingMapRight_1);
        knittingMapLeft.emplace_back(knittingMapLeft_1);
        for(int j = 0;j<knittingMapRight[i].size()-1;j++)
        {
            if(knittingMapRight[i][j]==-1&&knittingMapRight[i][j+1]==-1)
                continue;
            else if(knittingMapRight[i][j]==-1&&knittingMapRight[i][j+1]!=-1)
                knittingMapRight[i][j]=knittingMapRight[i][j+1]-1;

        }
        for(int j = 0;j<knittingMapLeft[i].size();j++)
        {
            if(knittingMapLeft[i][j]==-1&&knittingMapLeft[i][j+1]==-1)
                continue;
            else if(knittingMapLeft[i][j]==-1&&knittingMapLeft[i][j+1]!=-1)
                knittingMapLeft[i][j]=knittingMapLeft[i][j+1]-1;
        }
    }

    int maxLength = 0;
    for(int i = 0;i<knittingMapRight.size();i++)
    {
        if(knittingMapRight[i].size()>maxLength)
            maxLength = knittingMapRight[i].size();
    }
    for(int i = 0;i<knittingMapRight.size();i++)
    {
        int initialLength = knittingMapRight[i].size();
        if(initialLength<maxLength)
            for(int j =0;j<maxLength-initialLength;j++)
                knittingMapRight[i].emplace_back(-1);
    }
    for(int i = 0;i<knittingMapLeft.size();i++)
    {
        int initialLength = knittingMapLeft[i].size();
        if(initialLength<maxLength)
            for(int j =0;j<maxLength-initialLength;j++)
                knittingMapLeft[i].emplace_back(-1);
    }
    qDebug()<<"---------------------Map---------------------";
    for(int i =0;i<nodeLength.size();i++)
    {
        qDebug()<<i<<knittingMapLeft[i].size()<<knittingMapLeft[i];
        qDebug()<<i<<knittingMapRight[i].size()<<knittingMapRight[i];
    }
    std::vector<std::vector<int>> result_1;
    result_1.resize(knittingMapRight.size()-1);
    for(int i=0;i<result_1.size();i++)
    {
        result_1[i].resize(maxLength-1);
        for(int j = 0;j<result_1[i].size();j++)
        {
            int a = knittingMapRight[i][j];
            int b = knittingMapRight[i][j+1];
            int x = knittingMapLeft[i+1][j];
            int y = knittingMapLeft[i+1][j+1];
            if(a<0||b<0||x<0||y<0||(a==b&&x==y))
            {
                result_1[i][j]=0;
                continue;
            }
            if(a==b||x==y)
            {
                result_1[i][j]=2;
                continue;
            }
            result_1[i][j]=1;
        }
        //qDebug()<<i<<result_1[i];
    }
    result.resize(result_1[0].size());
    for(int i =0;i<result.size();i++)
    {
        for(int j = 0;j<result_1.size();j++)
            result[i].emplace_back(result_1[j][i]);
       // qDebug()<<i<<result[i];
    }

    std::ofstream myout("../../knittingMapRight.txt");
    for(int i =0;i<result.size();i++)
    {
        for(int j = 0;j<result[i].size();j++)
        {
            if(j==result[i].size()-1)
                myout<<result[i][j];
            else
                myout<<result[i][j]<<" ";
        }
        myout<<std::endl;
    }
    myout.close();


    return true;
}

bool LP_Plugin_Singa_Knitting::knittingMapgeneration_new()
{
    knittingMapMode = true;
    std::vector<std::vector<int>> knittingMap;
    std::vector<std::vector<std::vector<int>>> nodeLength;
    std::vector<std::vector<std::vector<int>>> relatedNodeUp_new;
    std::vector<std::vector<std::vector<int>>> relatedNodeDown_new;
    std::vector<std::vector<int>> result;
    std::vector<std::vector<int>> result_double;

    //re-assign the relatedNode
    for(int i = 0;i<relatedNode.size();i++)
    {
        std::vector<std::vector<int>> rNode;
        for(int j = 0;j<relatedNode[i].size();j++)
        {
            for(int k = 0;k<relatedNode[i][j].size();k++)
            {
                std::vector<int> rNode_1;
                for(int l = 0;l<relatedNode[i][j][k].size();l++)
                {
                    if(relatedNode[i][j][k][l][0]!=0&&i!=relatedNode.size()-1)
                    {
                        rNode_1.emplace_back(relatedNode[i][j][k][l][1]+relatedNode[i+1][relatedNode[i][j][k][l][0]-1].size());
                    }
                    else if(relatedNode[i][j][k][l][0]!=0&&i==relatedNode.size()-1)
                    {
                        rNode_1.emplace_back(relatedNode[i][j][k][l][1]+relatedNode_assist[i][relatedNode[i][j][k][l][0]-1].size());
                    }
                    else
                        rNode_1.emplace_back(relatedNode[i][j][k][l][1]);
                }
                rNode.emplace_back(rNode_1);
            }
        }
        relatedNodeUp_new.emplace_back(rNode);
    }
    for(int i = 0;i<relatedNode_assist.size();i++)
    {
        std::vector<std::vector<int>> rNode;
        for(int j = 0;j<relatedNode_assist[i].size();j++)
        {
            for(int k = 0;k<relatedNode_assist[i][j].size();k++)
            {
                std::vector<int> rNode_1;
                for(int l = 0;l<relatedNode_assist[i][j][k].size();l++)
                {
                    if(relatedNode_assist[i][j][k][l][0]!=0 && i!=0)
                        rNode_1.emplace_back(relatedNode_assist[i][j][k][l][1]+relatedNode_assist[i-1][relatedNode_assist[i][j][k][l][0]-1].size());
                    else if(relatedNode_assist[i][j][k][l][0]!=0&&i==0)
                    {
                        rNode_1.emplace_back(relatedNode_assist[i][j][k][l][1]+relatedNode[i][relatedNode[i][j][k][l][0]-1].size());
                    }
                    else
                        rNode_1.emplace_back(relatedNode_assist[i][j][k][l][1]);
                }
                rNode.emplace_back(rNode_1);
            }
        }
        relatedNodeDown_new.emplace_back(rNode);
    }
    //Delete isolated Node
    for(int i =0;i<relatedNodeUp_new.size();i++)
    {
        for(int j = 0;j<relatedNodeUp_new[i].size();j++)
        {
            if(relatedNodeUp_new[i][j].size()==0&&relatedNodeDown_new[i][j].size()==0)
            {
                qDebug()<<"There is an isolated Node--1";
                relatedNodeDown_new[i].erase(relatedNodeDown_new[i].begin()+j);
                relatedNodeUp_new[i].erase(relatedNodeUp_new[i].begin()+j);
            }
        }
    }
    //sort the Node
    for(int i =0;i<relatedNodeUp_new.size();i++)
    {
        for(int j = 0;j<relatedNodeUp_new[i].size();j++)
        {
            if(relatedNodeUp_new[i][j].size()<=1) continue;
            int a, b;
            for (a = 0; a < relatedNodeUp_new[i][j].size() - 1; a++)
                for (b = 0; b < relatedNodeUp_new[i][j].size() - 1 - a; b++)
                    if (relatedNodeUp_new[i][j][b] > relatedNodeUp_new[i][j][b + 1])
                        std::swap(relatedNodeUp_new[i][j][b], relatedNodeUp_new[i][j][b + 1]);
        }
        //qDebug()<<relatedNodeUp_new[i];
    }
    for(int i =0;i<relatedNodeDown_new.size();i++)
    {
        for(int j = 0;j<relatedNodeDown_new[i].size();j++)
        {
            if(relatedNodeDown_new[i][j].size()<=1) continue;
            int a, b;
            for (a = 0; a < relatedNodeDown_new[i][j].size() - 1; a++)
                for (b = 0; b < relatedNodeDown_new[i][j].size() - 1 - a; b++)
                    if (relatedNodeDown_new[i][j][b] > relatedNodeDown_new[i][j][b + 1])
                        std::swap(relatedNodeDown_new[i][j][b], relatedNodeDown_new[i][j][b + 1]);
        }
    }

    //Travel the Node(Get all branch included the disappeared branch);
    int firstLength = 0;
    for(int i = 0;i<relatedNodeUp_new.size();i++)
    {
        std::vector<std::vector<int>>nodeLength_1;
        for(int j = 0;j<relatedNodeUp_new[i].size();j++)
        {
            std::vector<int> branchNum;
            branchNum.resize(3);
            branchNum[0]=0;
            branchNum[1]=0;
            branchNum[2]=0;
            std::vector<int>nextNode;
            std::vector<int>nextNode_last;
            for(int k = 0;k<relatedNodeUp_new[i][j].size();k++)
                nextNode.emplace_back(relatedNodeUp_new[i][j][k]);
            std::set<int>s(nextNode.begin(), nextNode.end());
            nextNode.assign(s.begin(), s.end());
            nextNode_last.emplace_back(j);
            if(relatedNodeDown_new[i][j].size()>1) branchNum[1]+=relatedNodeDown_new[i][j].size()-1;
            if(relatedNodeUp_new[i][j].size()==0&&i!=relatedNodeUp_new.size()-1)
            {
                branchNum[1]+=1;
                branchNum[2] =1;
            }
            else if(relatedNodeUp_new[i][j].size()==0&&i==relatedNodeUp_new.size()-1)
            {
                branchNum[1]+=1;
                branchNum[2] =1;
            }
            for(int a =i+1;a<relatedNodeUp_new.size();a++)
            {
                std::vector<int>nextNode_new;
                nextNode_new.clear();
                for(int k = 0;k<nextNode.size();k++)
                {

                    if(relatedNodeUp_new[a][nextNode[k]].size()==0) branchNum[0]++;
                    if(relatedNodeDown_new[a][nextNode[k]].size()>1)
                    {
                        for(int l = 0;l<relatedNodeDown_new[a][nextNode[k]].size();l++)
                        {
                            for(int m = 0;m<nextNode_last.size();m++)
                            {
                                if(nextNode_last[m]==relatedNodeDown_new[a][nextNode[k]][l]) branchNum[0]++;
                            }
                        }
                        branchNum[0]--;
                    }
                    for(int l = 0;l<relatedNodeUp_new[a][nextNode[k]].size();l++)
                        nextNode_new.emplace_back(relatedNodeUp_new[a][nextNode[k]][l]);
                }
                std::set<int>s1(nextNode_new.begin(), nextNode_new.end());
                nextNode_new.assign(s1.begin(), s1.end());
                nextNode_last.clear();
                nextNode_last = nextNode;
                nextNode.clear();
                nextNode=nextNode_new;
            }
            nodeLength_1.emplace_back(branchNum);
        }
        if(i==0)
            for(int j = 0;j<nodeLength_1.size();j++)
                firstLength+=nodeLength_1[j][0];
        nodeLength.emplace_back(nodeLength_1);
    }
    for(int i = 0;i<relatedNodeUp_new.size();i++)
    {
        int idx_node= 0;
        int totalLength= 0;
        for(int j = 0;j<nodeLength[i].size();j++)
        {
            totalLength+=nodeLength[i][j][0];
            totalLength+=nodeLength[i][j][1];
        }
        if(totalLength==firstLength) continue;
        std::vector<std::vector<int>>nodeLength_1;
        //qDebug()<<"No."<<i;
        for(int j = 0;j<firstLength;j++)
        {
            int idx_last=0;
            int idx_last_node = 0;
            std::vector<int> branchNum_last;
            branchNum_last.resize(3);
            branchNum_last[0]=999;
            bool isFound = false;
            for(int k=0;k<nodeLength[i-1].size();k++)
            {
                for(int l = 0;l<nodeLength[i-1][k][0];l++)
                {
                    if(j==idx_last)
                    {
                        isFound = true;
                        break;
                    }
                    idx_last++;
                }
                if(isFound)
                {
                    branchNum_last = nodeLength[i-1][k];
                    idx_last_node = k;
                    break;
                }
                for(int l = 0;l<nodeLength[i-1][k][1];l++)
                {
                    if(j==idx_last)
                    {
                        isFound = true;
                        break;
                    }
                    idx_last++;
                }
                if(isFound)
                {
                    branchNum_last = nodeLength[i-1][k];
                    idx_last_node = k;
                    break;
                }
            }
            if(idx_node>nodeLength[i].size()-1)
            {
                std::vector<int> lastBranchNum;
                lastBranchNum.resize(3);
                lastBranchNum[0] = 0;
                for(int k = 0;k<nodeLength_1.size();k++)
                {
                    lastBranchNum[1]+=nodeLength_1[k][0]+nodeLength_1[k][1];
                }
                lastBranchNum[1] = firstLength-lastBranchNum[1];
                lastBranchNum[2] = 0;
                nodeLength_1.emplace_back(lastBranchNum);
                break;
            }
            if(branchNum_last[0]==0)
            {
                nodeLength_1.emplace_back(branchNum_last);
                j+=branchNum_last[1]-1;
            }
            else
            {
                std::vector<int> nodeLength_2;
                nodeLength_2.resize(3);
                nodeLength_2[0] = 0;
                nodeLength_2[1] = 0;
                nodeLength_2[2] = 0;
                nodeLength_2[0] = nodeLength[i][idx_node][0];
                if(relatedNodeDown_new[i][idx_node].size()>1)
                {
                    nodeLength_2[1]+=relatedNodeDown_new[i][idx_node].size()-1;
                    for(int k = 0;k<relatedNodeDown_new[i][idx_node].size();k++)
                    {
                        if(nodeLength[i-1][idx_last_node+k][2]==0)
                        {
                            nodeLength_2[1]+=nodeLength[i-1][idx_last_node+k][1];
                            nodeLength[i-1][idx_last_node+k][2] = 1;
                        }
                    }
                }
                if(relatedNodeUp_new[i][idx_node].size()==0&&i!=relatedNodeUp_new.size())
                {
                    nodeLength_2[1]+=1;
                    nodeLength_2[2] = 1;
                }
                else if(relatedNodeUp_new[i][idx_node].size()==0&&i==relatedNodeUp_new.size())
                {
                    nodeLength_2[1]+=1;
                    nodeLength_2[2] = 1;
                }
                if(nodeLength[i-1][idx_last_node][2]==0)
                {
                    nodeLength_2[1]+=branchNum_last[1];
                    nodeLength[i-1][idx_last_node][2] = 1;
                }
                j+=nodeLength_2[0]+nodeLength_2[1]-1;
                nodeLength_1.emplace_back(nodeLength_2);
                idx_node++;
            }
        }
        nodeLength.erase(nodeLength.begin()+i);
        nodeLength.insert(nodeLength.begin()+i,nodeLength_1);

    }
    for(int i = 0;i<relatedNodeUp_new.size();i++)
    {
        int totalLength= 0;
        for(int j = 0;j<nodeLength[i].size();j++)
        {
            totalLength+=nodeLength[i][j][0];
            totalLength+=nodeLength[i][j][1];
        }
        qDebug()<<i<<nodeLength[i]<<totalLength;
    }
    for(int i = 0;i<nodeLength.size();i++)
    {
        int idx = 0;
        std::vector<int> knittingMap_1;
        for(int j = 0;j<nodeLength[i].size();j++)
        {
            if(j == nodeLength[i].size()-1&&nodeLength[i][j][0]==0)
                for(int k = 0;k<nodeLength[i][j][0]+nodeLength[i][j][1];k++)
                {
                    knittingMap_1.emplace_back(knittingMap_1[knittingMap_1.size()-1]);
                }
            else
                for(int k = 0;k<nodeLength[i][j][0]+nodeLength[i][j][1];k++)
                {
                    knittingMap_1.emplace_back(idx);
                }
            if(nodeLength[i][j][0]!=0)
                idx++;
        }
        qDebug()<<i<<knittingMap_1;
        knittingMap.emplace_back(knittingMap_1);
    }
    std::vector<std::vector<int>> result_1;
    result_1.resize(knittingMap.size()-1);
    for(int i=0;i<result_1.size();i++)
    {
        result_1[i].resize(firstLength-1);
        for(int j = 0;j<result_1[i].size();j++)
        {
            int a = knittingMap[i][j];
            int b = knittingMap[i][j+1];
            int x = knittingMap[i+1][j];
            int y = knittingMap[i+1][j+1];
            if(a<0||b<0||x<0||y<0||(a==b&&x==y))
            {
                result_1[i][j]=0;
                continue;
            }
            if(a==b||x==y)
            {
                result_1[i][j]=2;
                continue;
            }
            result_1[i][j]=1;
        }
    }
    result.resize(result_1[0].size()+2);

    for(int i =0;i<result.size();i++)
    {
        if(i==0||i == result.size()-1)
            for(int j = 0;j<result_1.size();j++)
                if(j==0||j==result_1.size()-1)
                    result[i].emplace_back(2);
                else
                    result[i].emplace_back(1);

        else
            for(int j = 0;j<result_1.size();j++)
                result[i].emplace_back(result_1[j][i-1]);
    }
    for(int i= 0;i<result.size();i++)
    {
        for(int j =0;j<result[i].size();j++)
        {
            if(result[i][j]!=0)
            {
                if((j!=0&&j!=result[i].size()-1)&&(result[i][j+1]==0||result[i][j-1]==0))
                    result[i][j]=2;
                else if(j==0||j==result[i].size()-1) result[i][j]=2;
            }
        }
        qDebug()<<i<<result[i];
    }

    for(int i =0;i<result.size();i++)
    {
        result_double.emplace_back(result[i]);
        result_double.emplace_back(result[i]);
    }

//---------------------------post-process-------------------------------//

    for(int i =0;i<result_double.size()-1;i++)
    {
        bool isFound = false;
        if(i%2 == 0)
        {
            for(int j=0;j<result_double[i].size();j++)
            {
                if(result_double[i][j]!=0)
                {
                    if(j!=result_double[i].size()-1)
                    {
                        if(result_double[i+1][j]!=0&&result_double[i+1][j+1]==0)
                        {
                            result_double[i][j]=2;
                            for(int k = j+1;k<result_double[i].size();k++)
                                result_double[i][k]=0;
                            isFound = true;
                        }
                    }
                    else
                        result_double[i][j]=2;
                }
                if(isFound) break;
            }
            if(!isFound)
            {
                for(int j=0;j<result_double[i].size();j++)
                {
                    if(j!=result_double[i].size()-1 && result_double[i][j]!=0 && result_double[i][j+1]==0)
                    {
                        for(int k = j;k<result_double[i].size();k++)
                        {
                            if(result_double[i+1][k]==0)
                            {
                                result_double[i][k-1] = 2;
                                break;
                            }
                            result_double[i][k]=1;
                            if(k==result_double[i].size()-1)result_double[i][k]=2;
                        }
                        break;
                    }
                    else if(j==result_double[i].size()-1) result_double[i][j]=2;
                }
            }
        }
        else
        {
            for(int j=result_double[i].size()-1;j>-1;j--)
            {
                if(result_double[i][j]!=0)
                {
                    if(j!=0)
                    {
                        //qDebug()<<i<<"G1";
                        if(result_double[i+1][j]!=0 && result_double[i+1][j-1]==0)
                        {
                            //qDebug()<<i<<"GG";
                            result_double[i][j]=2;
                            for(int k = j-1;k>-1;k--)
                                result_double[i][k]=0;
                            isFound = true;
                        }
                    }
                    else
                    {
                        result_double[i][j]=2;
                        //qDebug()<<i<<"G2";
                    }
                }
                if(isFound) break;
            }
            if(!isFound)
            {
                for(int j=result_double[i].size()-1;j>-1;j--)
                {
                    if(j!=0 && result_double[i][j]!=0 && result_double[i][j-1]==0)
                    {
                        for(int k = j;k>-1;k--)
                        {
                            if(result_double[i+1][k]==0)
                            {
                                result_double[i][k+1] = 2;
                                break;
                            }
                            result_double[i][k]=1;
                            if(k==0)result_double[i][k]=2;
                        }
                        break;
                    }
                    else if(j==0) result_double[i][j]=2;
                }
            }
        }
    }
    std::ofstream myout("../../knittingMapSingle.txt");
    for(int i =0;i<result.size();i++)
    {
        for(int j = 0;j<result[i].size();j++)
        {
            if(j==result[i].size()-1)
                myout<<result[i][j];
            else
                myout<<result[i][j]<<" ";
        }
        myout<<std::endl;
    }
    myout.close();

    std::ofstream myout_double("../../knittingMapDouble.txt");
    for(int i =0;i<result_double.size();i++)
    {
        for(int j = 0;j<result_double[i].size();j++)
        {
            if(j==result_double[i].size()-1)
                myout_double<<result_double[i][j];
            else
                myout_double<<result_double[i][j]<<" ";
        }
        myout_double<<std::endl;
    }
    myout_double.close();

    knittingMapArray = result_double;
    return true;
}

void LP_Plugin_Singa_Knitting::FunctionalRender_L(QOpenGLContext *ctx, QSurface *surf, QOpenGLFramebufferObject *fbo, const LP_RendererCam &cam, const QVariant &options)
{
    Q_UNUSED(surf)  //Mostly not used within a Functional.
    Q_UNUSED(options)   //Not used in this functional.

    mCam = cam;

    if ( !mInitialized ){   //The OpenGL resources, e.g. Shader, not initilized
        initializeGL();     //Call the initialize member function.
    }                       //Not compulsory, (Using member function for cleaness only)
    if ( !mObject.lock()){
        return;             //If not mesh is picked, return. mObject is a weak pointer
    }                       //to a LP_OpenMesh.

    auto proj = cam->ProjectionMatrix(),    //Get the projection matrix of the 3D view
         view = cam->ViewMatrix();          //Get the view matrix of the 3D view

    auto f = ctx->extraFunctions();         //Get the OpenGL functions container

    fbo->bind();
    f->glEnable(GL_PROGRAM_POINT_SIZE);     //Enable point-size controlled by shader
    f->glEnable(GL_DEPTH_TEST);             //Enable depth test

    mProgram->bind();                       //Bind the member shader to the context
    mProgram->setUniformValue("m4_mvp", proj * view );  //Set the Model-View-Projection matrix
    mProgram->setUniformValue("f_pointSize", 7.0f);     //Set the point-size which is enabled before
    mProgram->setUniformValue("v4_color", QVector4D( 0.4f, 0.7f, 0.7f, 0.6f )); //Set the point color

//    Get the actual open-mesh data from the LP_OpenMesh class
    auto m = std::static_pointer_cast<LP_OpenMeshImpl>(mObject.lock())->Mesh();

    mProgram->enableAttributeArray("a_pos");        //Enable the "a_pos" attribute buffer of the shader
    mProgram->setAttributeArray("a_pos",m->points()->data(),3); //Set the buffer data of "a_pos"

    if(selectMode)
        f->glDrawArrays(GL_POINTS, 0, GLsizei(m->n_vertices()));



    f->glLineWidth(10.0f);
    if(!isoCurveNode.empty() &&isoCurveNode_resampled.empty())
    {
        for(int i = 0;i<isoCurveNode.size();i++)
        {
            int sum = 0;int count = 0;
            for(int j = 0;j<isoCurveNode[i].size();j++)
                sum+=isoCurveNode[i][j].size();
            for(int j = 0;j<isoCurveNode[i].size();j++)
            {
                for(int k = 0 ;k<isoCurveNode[i][j].size()-1;k++)
                {
                    std::vector<float> path;
                    for(int l = 0;l<3;l++) path.push_back(isoCurveNode[i][j][k][l]);
                    for(int l = 0;l<3;l++) path.push_back(isoCurveNode[i][j][k+1][l]);
                    mProgram->setUniformValue("v4_color", QVector4D( float((k+count)/float(sum))*1.0f, float((k+count)/float(sum))*1.0f, float((k+count)/float(sum))*1.0f, 1.0f ));
                    mProgram->setAttributeArray("a_pos",path.data(),3);
                    f->glDrawArrays(GL_LINE_STRIP, 0, GLsizei(2));
                }
                count+=isoCurveNode[i][j].size();
            }
        }
    }


    if(!isoCurveNode_resampled.empty())
    {
        for(int i = 0;i<isoCurveNode_resampled.size();i++)
        {
            int sum = 0;int count = 0;
            for(int j = 0;j<isoCurveNode_resampled[i].size();j++)
                sum+=isoCurveNode_resampled[i][j].size();
            for(int j = 0;j<isoCurveNode_resampled[i].size();j++)
            {
                for(int k = 0 ;k<isoCurveNode_resampled[i][j].size()-1;k++)
                {
                    std::vector<float> path;
                    for(int l = 0;l<3;l++) path.push_back(isoCurveNode_resampled[i][j][k][l]);
                    for(int l = 0;l<3;l++) path.push_back(isoCurveNode_resampled[i][j][k+1][l]);
                    //mProgram->setUniformValue("v4_color", QVector4D( float(k/float(isoCurveNode[i][j].size()))*1.0f, float(k/float(isoCurveNode[i][j].size()))*1.0f, float(k/float(isoCurveNode[i][j].size()))*1.0f, 1.0f ));
                    mProgram->setUniformValue("v4_color", QVector4D( float((k+count)/float(sum))*1.0f, float((k+count)/float(sum))*0.0f, float((k+count)/float(sum))*0.0f, 1.0f ));
                    mProgram->setAttributeArray("a_pos",path.data(),3);
                    f->glDrawArrays(GL_LINE_STRIP, 0, GLsizei(2));
                }
                count+=isoCurveNode_resampled[i][j].size();
            }
        }
    }

    mProgram->setUniformValue("v4_color", QVector4D( 0.4f, 0.7f, 0.7f, 0.6f ));
    if(!relatedNode.empty())
    {
        for(int i = 0;i<relatedNode.size();i++)
        {
            for(int j = 0;j<relatedNode[i].size();j++)
            {
                for(int k = 0 ;k<relatedNode[i][j].size();k++)
                {
                    for(int l = 0;l<relatedNode[i][j][k].size();l++)
                    {
                        std::vector<float> path;
                        for(int m = 0;m<3;m++) path.push_back(isoCurveNode_resampled[i][j][k][m]);
                        for(int m = 0;m<3;m++) path.push_back(isoCurveNode_resampled[i+1][relatedNode[i][j][k][l][0]][relatedNode[i][j][k][l][1]][m]);
                        mProgram->setAttributeArray("a_pos",path.data(),3);
                        f->glDrawArrays(GL_LINE_STRIP, 0, GLsizei(2));
                    }
                }
            }
        }
    }
    if (!mPoints.empty()&& selectMode){                         //If some vertices are picked and record in mPoints
        mProgram->setUniformValue("f_pointSize", 13.0f);    //Enlarge the point-size
        mProgram->setUniformValue("v4_color", QVector4D( 1.0f, 0.7f, 0.8f, 1.0f )); //Change to another color

        std::vector<uint> list(mPoints.size());
        for ( int i = 0; i<list.size(); ++i ){
            list[i] = (mPoints.begin()+i).key();
        }
        // ONLY draw the picked vertices again
        f->glDrawElements(GL_POINTS, GLsizei(mPoints.size()), GL_UNSIGNED_INT, list.data());
    }
    if(field_color.empty()&&fieldMode)
    {
        fieldMode = false;
        auto mesh = std::static_pointer_cast<LP_OpenMeshImpl>(mObject.lock());
        QVariant opt = options;
        typename OpMesh::ConstVertexIter vIt(m->vertices_begin());
        typename OpMesh::ConstVertexIter vEnd(m->vertices_end());
        OpMesh::Color vcolor(0,0,0);
        int i = 0;
        for (vIt = m->vertices_begin(); vIt!=vEnd; ++vIt){
            vcolor[0] = ocolor[i*3]*255.0f;
            vcolor[1] = ocolor[i*3+1]*255.0f;
            vcolor[2] = ocolor[i*3+2]*255.0f;
            m->set_color(*vIt, vcolor);
            i++;
        }
        labelMaxDis->setText("NA");
        mesh->DrawCleanUp(ctx, surf);
        mesh->DrawSetup(ctx, surf, opt);
        ctx->makeCurrent(surf);
    }
    if (!field_color.empty()&&!fieldMode ){   // Draw the distance field
        auto mesh = std::static_pointer_cast<LP_OpenMeshImpl>(mObject.lock());

        QVariant opt = options;

        typename OpMesh::ConstVertexIter vIt(m->vertices_begin());
        typename OpMesh::ConstVertexIter vEnd(m->vertices_end());

        if(ocolor.empty()){ // Save the original colors
        auto cptr = m->vertex_colors();

            for ( size_t i=0; i< m->n_vertices(); ++i, ++cptr ){

            const uchar *_p = cptr->data();
            ocolor.push_back(_p[0] / 255.0f);
            ocolor.push_back(_p[1] / 255.0f);
            ocolor.push_back(_p[2] / 255.0f);
            }
        }
        OpMesh::Color vcolor(0,0,0);
        int i = 0;
        for (vIt = m->vertices_begin(); vIt!=vEnd; ++vIt){
            vcolor[0] = field_color[i*3]*255.0f;
            vcolor[1] = field_color[i*3+1]*255.0f;
            vcolor[2] = field_color[i*3+2]*255.0f;
            m->set_color(*vIt, vcolor);
            i++;
        }

            mesh->DrawCleanUp(ctx, surf);
            mesh->DrawSetup(ctx, surf, opt);
            ctx->makeCurrent(surf);
            fieldMode = true;

    }
    else if(!ocolor.empty() && field_color.empty()){ // Draw the original colors
        auto mesh = std::static_pointer_cast<LP_OpenMeshImpl>(mObject.lock());
        QVariant opt = options;
        mesh->DrawCleanUp(ctx, surf);
        mesh->DrawSetup(ctx, surf, opt);
        typename OpMesh::ConstVertexIter vIt(m->vertices_begin());
        typename OpMesh::ConstVertexIter vEnd(m->vertices_end());
        OpMesh::Color vcolor(0,0,0);
        int i = 0;
        for (vIt = m->vertices_begin(); vIt!=vEnd; ++vIt){
            vcolor[0] = ocolor[i*3]*255.0f;
            vcolor[1] = ocolor[i*3+1]*255.0f;
            vcolor[2] = ocolor[i*3+2]*255.0f;
            m->set_color(*vIt, vcolor);
            i++;
        }
        ctx->makeCurrent(surf);
    }

    mProgram->disableAttributeArray("a_pos");   //Disable the "a_pos" buffer

    mProgram->release();                        //Release the shader from the context

    fbo->release();
    f->glDisable(GL_PROGRAM_POINT_SIZE);
    f->glDisable(GL_DEPTH_TEST);

}

void LP_Plugin_Singa_Knitting::FunctionalRender_R(QOpenGLContext *ctx, QSurface *surf, QOpenGLFramebufferObject *fbo, const LP_RendererCam &cam, const QVariant &options)
{
    if(!knittingMapMode) return;
    Q_UNUSED(cam)
    Q_UNUSED(surf)  //Mostly not used within a Functional.
    Q_UNUSED(options)   //Not used in this functional.

    if ( !mInitialized ||true ){   //The OpenGL resources, e.g. Shader, not initilized
        initializeGL();     //Call the initialize member function.

        mImage = QImage(0.1*fbo->width(), 0.1*fbo->height(), QImage::Format_RGB888);
        mImage.fill(Qt::black);
        qDebug() << "FBO : " << fbo->width() << "x" << fbo->height();
    }                       //Not compulsory, (Using member function for cleaness only)



    static auto rainbow = [](int p, int np, float&r, float&g, float&b) {    //16,777,216
            float inc = 6.0 / np;
            float x = p * inc;
            r = 0.0f; g = 0.0f; b = 0.0f;
            if ((0 <= x && x <= 1) || (5 <= x && x <= 6)) r = 1.0f;
            else if (4 <= x && x <= 5) r = x - 4;
            else if (1 <= x && x <= 2) r = 1.0f - (x - 1);
            if (1 <= x && x <= 3) g = 1.0f;
            else if (0 <= x && x <= 1) g = x - 0;
            else if (3 <= x && x <= 4) g = 1.0f - (x - 3);
            if (3 <= x && x <= 5) b = 1.0f;
            else if (2 <= x && x <= 3) b = x - 2;
            else if (5 <= x && x <= 6) b = 1.0f - (x - 5);
        };

    auto f = ctx->extraFunctions();         //Get the OpenGL functions container

    f->glEnable(GL_DEPTH_TEST);             //Enable depth test

    fbo->bind();
    mProgram->bind();                       //Bind the member shader to the context

    QOpenGLTexture tex_(mImage);
    tex_.setMinMagFilters(QOpenGLTexture::Nearest,QOpenGLTexture::Nearest);
    tex_.create();
    tex_.bind();

    mProgram->setUniformValue("u_tex", 0);
    mProgram->setUniformValue("v4_color", QVector4D(1.0f,1.0f,1.0f,1.0f));
    mProgram->setUniformValue("m4_mvp", QMatrix4x4() );

    mProgram->enableAttributeArray("a_pos");
    mProgram->enableAttributeArray("a_tex");

    mVBO->bind();
    mIndices->bind();

    mProgram->setAttributeBuffer("a_pos",GL_FLOAT, 0, 3, 20);
    mProgram->setAttributeBuffer("a_tex",GL_FLOAT, 12, 2, 20);

    f->glDrawElements(  GL_TRIANGLE_STRIP,
                        4,
                        GL_UNSIGNED_INT,
                        0);//Actually draw all the image

    mVBO->release();
    mIndices->release();
    tex_.release();
    tex_.destroy();

    mProgram->disableAttributeArray("a_tex");

    //Draw grid-lines
    f->glLineWidth(1.0f);

    qDebug()<<knittingMapArray[0].size()<<"*"<<knittingMapArray.size();
    qDebug()<<mImage.width()<<"*"<<mImage.height();
//    const float hStep = 2.0f / mImage.width(),      //In Screen space
//                vStep = 2.0f / mImage.height();
    const float hStep = 2.0f/knittingMapArray[0].size(),      //In Screen space
                vStep = 2.0f/knittingMapArray.size();
    qDebug()<<hStep<<"*"<<vStep;

    float r, g, b;

//    rainbow(mSlider->value(), mSlider->maximum(), r, g, b);
//    rainbow(2856582,16777215,r,g,b);
    r = 0.3f;g=0.3f;b=0.3f;
    mProgram->setUniformValue("v4_color", QVector4D(r, g, b, 1.0f));
    std::vector<QVector3D> l;
    for ( int i=0; i<=knittingMapArray[0].size(); ++i ){
        l.emplace_back(QVector3D( hStep*i - 1.0f, 1.0f, 0.1f ));
        l.emplace_back(QVector3D( hStep*i - 1.0f, -1.0f, 0.1f ));
    }
    for ( int j=0; j<=knittingMapArray.size(); ++j ){//Horizontal
        l.emplace_back(QVector3D( -1.0f, j*vStep - 1.0f, 0.1f ));
        l.emplace_back(QVector3D( 1.0f, j*vStep - 1.0f, 0.1f ));
    }

    f->glEnable(GL_BLEND);
    f->glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    std::vector<QVector3D> poly_red;
    std::vector<QVector3D> poly_blue;
    std::vector<QVector3D> poly_na;
    for(int i = 0;i<knittingMapArray.size();i++)
    {
        for(int j = 0;j<knittingMapArray[i].size();j++)
        {
            if(knittingMapArray[i][j]==2)
            {
                poly_red.emplace_back(QVector3D(-1.0f+hStep*j,-1.0f+vStep*i,0.1f));
                poly_red.emplace_back(QVector3D(-1.0f+hStep*j,-1.0f+vStep*(i+1),0.1f));
                poly_red.emplace_back(QVector3D(-1.0f+hStep*(j+1),-1.0f+vStep*(i+1),0.1f));
                poly_red.emplace_back(QVector3D(-1.0f+hStep*(j+1),-1.0f+vStep*i,0.1f));
            }
            else if(knittingMapArray[i][j]==1)
            {
                poly_blue.emplace_back(QVector3D(-1.0f+hStep*j,-1.0f+vStep*i,0.1f));
                poly_blue.emplace_back(QVector3D(-1.0f+hStep*j,-1.0f+vStep*(i+1),0.1f));
                poly_blue.emplace_back(QVector3D(-1.0f+hStep*(j+1),-1.0f+vStep*(i+1),0.1f));
                poly_blue.emplace_back(QVector3D(-1.0f+hStep*(j+1),-1.0f+vStep*i,0.1f));
            }
            else
            {
                poly_na.emplace_back(QVector3D(-1.0f+hStep*j,-1.0f+vStep*i,0.1f));
                poly_na.emplace_back(QVector3D(-1.0f+hStep*(j+1),-1.0f+vStep*(i+1),0.1f));
                poly_na.emplace_back(QVector3D(-1.0f+hStep*j,-1.0f+vStep*(i+1),0.1f));
                poly_na.emplace_back(QVector3D(-1.0f+hStep*(j+1),-1.0f+vStep*i,0.1f));
            }
        }
    }

    mProgram->setAttributeArray("a_pos", l.data());
    f->glDrawArrays(GL_LINES, 0, GLsizei(l.size()));
    mProgram->setUniformValue("v4_color", QVector4D(230.0f/255,65.0f/255,31.0f/255, 1.0f));
    mProgram->setAttributeArray("a_pos", poly_red.data());
    f->glDrawArrays(GL_QUADS,0,GLsizei(poly_red.size()));
    mProgram->setUniformValue("v4_color", QVector4D(155.0f/255,194.0f/255,230.0f/255, 1.0f));
    mProgram->setAttributeArray("a_pos", poly_blue.data());
    f->glDrawArrays(GL_QUADS,0,GLsizei(poly_blue.size()));
    mProgram->setUniformValue("v4_color", QVector4D(0.0f,0.0f,0.0f, 1.0f));
    mProgram->setAttributeArray("a_pos", poly_na.data());
    f->glDrawArrays(GL_LINES,0,GLsizei(poly_na.size()));
    f->glDisable(GL_BLEND);


    mProgram->disableAttributeArray("a_pos"); //Disable the "a_pos" buffer

    mProgram->release();                        //Release the shader from the context
    fbo->release();
    f->glDisable(GL_DEPTH_TEST);
}

bool LP_Plugin_Singa_Knitting::eventFilter(QObject *watched, QEvent *event)
{
    static auto _isMesh = [](LP_Objectw obj){
        if ( obj.expired()){
            return LP_OpenMeshw();
        }
        return LP_OpenMeshw() = std::static_pointer_cast<LP_OpenMeshImpl>(obj.lock());
    };
    if(selectMode)
    {
        if ( QEvent::MouseButtonRelease == event->type()){
            auto e = static_cast<QMouseEvent*>(event);
            if ( e->button() == Qt::LeftButton ){
                if (!mObject.lock()){
                    auto &&objs = g_GLSelector->SelectInWorld("Shade",e->pos());
                    for ( auto &o : objs ){
                        auto c = _isMesh(o);
                        if ( c.lock()){
                            mObject = o;
                            emit glUpdateRequest();
                            return true;    //Since event filter has been called
                        }
                    }
                }
                else{//Select the entity vertices
                    auto c = _isMesh(mObject).lock();
                    auto &&tmp = g_GLSelector->SelectPoints3D("Shade",
                                                        c->Mesh()->points()->data(),
                                                        c->Mesh()->n_vertices(),
                                                        e->pos(), true);

                    if ( !tmp.empty() && c->Mesh()->is_boundary(c->Mesh()->vertex_handle(tmp.front()))){
                        auto &&pt_id = tmp.front();
                        qDebug() << "Picked : " << tmp;
                        if (Qt::ControlModifier & e->modifiers()){
                            if ( !mPoints.contains(pt_id)){
                                mPoints.insert(pt_id, mPoints.size());
                            }
                        }else if (Qt::ShiftModifier & e->modifiers()){
                            if ( mPoints.contains(pt_id)){
                                qDebug() << mPoints.take(pt_id);
                            }
                        }else{
                            //mPoints.clear();
                            //mPoints.insert(pt_id,0);
                            if ( !mPoints.contains(pt_id))
                            mPoints.insert(pt_id, mPoints.size());
                        }

                        QString info(mObject.lock()->Uuid().toString());
                        for ( auto &p : mPoints ){
                            info += tr("%1\n").arg(p);
                        }

                        if (!mPoints.empty()){
                            emit glUpdateRequest();
                        }
                        //printf("The mesh is %B\n",c.get()->Mesh()->has_vertex_status());
                        //qDebug() << c.get()->Mesh()->vertex_handle(100).idx();
                        return true;
                    }
                }
            }
        }
    }
    return QObject::eventFilter(watched, event);
}

void LP_Plugin_Singa_Knitting::ResetSelection()
{

}

void LP_Plugin_Singa_Knitting::PainterDraw(QWidget *glW)
{
    if(!selectMode) return;
    if ( "openGLWidget_2" == glW->objectName()){
        return;
    }
    if ( !mCam.lock() || !mObject.lock()){
        return;
    }
    auto m = std::static_pointer_cast<LP_OpenMeshImpl>(mObject.lock())->Mesh();
    auto cam = mCam.lock();
    auto view = cam->ViewMatrix(),
         proj = cam->ProjectionMatrix(),
         vp   = cam->ViewportMatrix();

    view = vp * proj * view;
    auto &&h = cam->ResolutionY();
    QPainter painter(glW);
    int fontSize(13);
    QFont font("Arial", fontSize);
    QFontMetrics fmetric(font);
    QFont orgFont = painter.font();
    painter.setPen(qRgb(255,0,0));

    painter.setFont(font);

    for (auto it = mPoints.begin(); it != mPoints.end(); ++it ){
        auto vid = it.key();
        auto pickId = it.value();
        if ( vid > m->n_vertices()){
            continue;
        }
        auto pt = m->points()[vid];
        QVector3D v(pt[0], pt[1], pt[2]);
        v = view * v;
        painter.drawText(QPointF(v.x(), h-v.y()), QString("%1").arg(pickId));
    }

    painter.setFont(orgFont);
}

void LP_Plugin_Singa_Knitting::initializeGL()
{

    constexpr char vsh[] =
            "attribute vec3 a_pos;\n"       //The position of a point in 3D that used in FunctionRender()
            "uniform mat4 m4_mvp;\n"        //The Model-View-Matrix
            "uniform float f_pointSize;\n"  //Point size determined in FunctionRender()
            "void main(){\n"
            "   gl_Position = m4_mvp * vec4(a_pos, 1.0);\n" //Output the OpenGL position
            "   gl_PointSize = f_pointSize;\n"
            "}";
    constexpr char fsh[] =
            "uniform vec4 v4_color;\n"       //Defined the point color variable that will be set in FunctionRender()
            "void main(){\n"
            "   gl_FragColor = v4_color;\n" //Output the fragment color;
            "}";

    auto prog = new QOpenGLShaderProgram;   //Intialize the Shader with the above GLSL codes
    prog->addShaderFromSourceCode(QOpenGLShader::Vertex,vsh);
    prog->addShaderFromSourceCode(QOpenGLShader::Fragment,fsh);
    if (!prog->create() || !prog->link()){  //Check whether the GLSL codes are valid
        qDebug() << prog->log();
        return;
    }

    mProgram = prog;            //If everything is fine, assign to the member variable
//---------------------------------------------------------------------------------------------------
    //Assume full screen plane in normalized projection space [-1,1]x[-1,1]x[-1,1] with fixed depth
    std::vector<QVector3D> pos = {QVector3D(1.0f,-1.0f,0.9f),
                                      QVector3D(1.0f, 1.0f, 0.9f),
                                      QVector3D(-1.0f, -1.0f, 0.9f),
                                      QVector3D(-1.0f, 1.0f, 0.9f)};

    //The corresponding uv-coord
    std::vector<QVector2D> texCoord = {QVector2D(1.0f,1.0f),
                                      QVector2D(1.0f, 0.0f),
                                      QVector2D(0.0f, 1.0f),
                                      QVector2D(0.0f, 0.0f)};

    const size_t nVs = pos.size();

    QOpenGLBuffer *posBuf = new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer);
    posBuf->setUsagePattern(QOpenGLBuffer::StreamDraw);
    posBuf->create();
    posBuf->bind();
    posBuf->allocate(int( nVs * ( sizeof(QVector2D) + sizeof(QVector3D))));
    //mVBO->allocate( m->points(), int( m->n_vertices() * sizeof(*m->points())));
    auto ptr = static_cast<float*>(posBuf->map(QOpenGLBuffer::WriteOnly));
    auto pptr = pos.begin();
    auto tptr = texCoord.begin();
    for ( size_t i=0; i<nVs; ++i, ++pptr, ++tptr ){
        memcpy(ptr, &(*pptr)[0], sizeof(QVector3D));
        memcpy(ptr+3, &(*tptr)[0], sizeof(QVector2D));
        ptr = std::next(ptr, 5);
        //qDebug() << QString("%1, %2, %3").arg((*nptr)[0]).arg((*nptr)[1]).arg((*nptr)[2]);
    }
    posBuf->unmap();
    posBuf->release();

    mVBO = posBuf;

    const std::vector<uint> indices = {0, 1, 2, 3};

    QOpenGLBuffer *indBuf = new QOpenGLBuffer(QOpenGLBuffer::IndexBuffer);
    indBuf->setUsagePattern(QOpenGLBuffer::StaticDraw);

    indBuf->create();
    indBuf->bind();
    indBuf->allocate(int( indices.size() * sizeof(indices[0])));
    auto iptr = static_cast<uint*>(indBuf->map(QOpenGLBuffer::WriteOnly));
    memcpy(iptr, indices.data(), indices.size() * sizeof(indices[0]));
    indBuf->unmap();
    indBuf->release();

    mIndices = indBuf;
//---------------------------------------------------------------------------------------------------


    mInitialized = true;
}


QString LP_Plugin_Singa_Knitting::MenuName()
{
    return tr("menuPlugins");
}


QAction *LP_Plugin_Singa_Knitting::Trigger()
{
    if ( !mAction ){
        mAction = new QAction("Singa Knitting");
    }
    return mAction;
}
