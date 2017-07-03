//
// Created by carlotta on 26/06/17.
//

#include "MemoryManager.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include <sstream>
#include "pitt_msgs/TrackedShapes.h"
#include "pitt_msgs/TrackedShape.h"
#include <ros/console.h>
#include "memory_library/memoryManager.h"
#include "sit_msgs/EpisodicInterface.h"
#include "sit_msgs/SemanticInterface.h"
#include "sit_msgs/ScoreInterface.h"
#include <vector>
using namespace pitt_msgs;
using namespace ros;
using namespace sensor_msgs;
using namespace std;
using namespace memManager;
using namespace sit_msgs;

TrackedShapes correctedRansacShapes;
bool receivedNewShapes=false;
bool processPittInfo= false;

vector<float> coefficientsFromRansacToSemantic(TrackedShape shape);

void ransac_shape_acquisition(TrackedShapes RansacShapes){
    if(processPittInfo) {

        for (int i = 0; i < RansacShapes.tracked_shapes.size(); i++) {
            TrackedShape shape = RansacShapes.tracked_shapes[i];
            cout << shape.object_id << endl;
            cout << shape.shape_tag << endl;
            cout << "x centroid " << shape.x_pc_centroid << endl;
            cout << "y centroid " << shape.y_pc_centroid << endl;
            cout << "z centroid " << shape.z_pc_centroid << endl;

            cout << "coefficients" << endl;
            for (int j = 0; j < shape.coefficients.size(); j++) {
                cout << shape.coefficients[j] << endl;
            }
            cout << "color " << shape.color.data << endl;

        }
        if(userCheck()){
            correctedRansacShapes=RansacShapes;
        }
        else {
            if (userCheckContinue("obtain info from pitt")) {
                receivedNewShapes=false;
                processPittInfo=true;
                return ;
            }
            else {
                cout << "correct the shape" << endl;
                for (int i = 0; i < RansacShapes.tracked_shapes.size(); i++) {
                    TrackedShape shape = RansacShapes.tracked_shapes[i];
                    TrackedShape shapeNew;
                    cout << "shapeId" << endl;
                    cout << shape.object_id << endl;
                    cout << "shapeTag" << endl;
                    cout << shape.shape_tag << endl;
                    cout << "type new shape tag" << endl;
                    string newShapeTag;
                    getline(cin, newShapeTag);
                    if (newShapeTag == SPHERE ||
                        newShapeTag == CONE ||
                        newShapeTag == CYLINDER ||
                        newShapeTag == PLANE) {
                        shapeNew.object_id = shape.object_id;
                        shapeNew.shape_tag = newShapeTag;
                        if (userCheckValues("centroids information")) {
                            float newXCentroid;
                            cout << "x centroid " << shape.x_pc_centroid << endl;
                            cout << "insert new x centroid" << endl;
                            cin >> newXCentroid;
                            shapeNew.x_pc_centroid = newXCentroid;
                            float newYCentroid;
                            cout << "y centroid " << shape.y_pc_centroid << endl;
                            cout << "insert new y centroid" << endl;
                            cin >> newYCentroid;
                            shapeNew.y_pc_centroid = newYCentroid;
                            float newZCentroid;
                            cout << "z centroid " << shape.z_pc_centroid << endl;
                            cout << "insert new z centroid" << endl;
                            cin >> newZCentroid;
                            shapeNew.z_pc_centroid = newYCentroid;
                        } else {
                            shapeNew.x_pc_centroid = shape.x_pc_centroid;
                            shapeNew.y_pc_centroid = shape.y_pc_centroid;
                            shapeNew.z_pc_centroid = shape.z_pc_centroid;
                        }
                        if (newShapeTag == shape.shape_tag) {
                            if (userCheckValues("coefficients information")) {
                                vector<float> coefficientNew;
                                for (int j = 0; j < shape.coefficients.size(); j++) {
                                    float newCoefficient;
                                    cout << shape.coefficients[j] << endl;
                                    cout << "insert new coefficinet" << endl;
                                    cin >> newCoefficient;
                                    coefficientNew.push_back(newCoefficient);
                                }
                                shapeNew.coefficients = coefficientNew;
                            } else {
                                shapeNew.coefficients = shape.coefficients;
                            }
                        } else {
                            vector<float> coefficientNew;
                            cout << "insert new coefficients" << endl;
                            bool addOtherElements;
                            do {
                                float coeff;
                                cout << "insert coefficient" << endl;
                                cin >> coeff;
                                coefficientNew.push_back(coeff);
                                addOtherElements = userCheckContinue("adding coefficients");
                            } while (addOtherElements);
                            shapeNew.coefficients = coefficientNew;


                        }

                        if (userCheckValues("color")) {
                            string newColor;
                            cout << "color " << shape.color.data << endl;
                            cout << "insert new color " << endl;
                            getline(cin, newColor);
                            shapeNew.color.data = newColor;
                        } else {
                            shapeNew.color.data = shape.color.data;
                        }
                        correctedRansacShapes.tracked_shapes.push_back(shapeNew);

                    }


                }
                receivedNewShapes = true;
            }
            processPittInfo = false;
        }
    }


}

int main(int argc, char **argv) {
    ros::init(argc, argv, MEMORIZATION_MANAGER_NODE_NAME);
    ros::NodeHandle node;
    ros::NodeHandle* nodePtr=&node;
    // set subscriber to get perceived shape informations
    ros::Subscriber subPITT = node.subscribe(PITT_TOPIC,1000, ransac_shape_acquisition);
    //declaration of the services
    ServiceClient client_semantic = nodePtr->serviceClient < SemanticInterface > (SRV_SEMANTIC);
    SemanticInterface srv_semantic;
    ServiceClient client_episodic = nodePtr->serviceClient < EpisodicInterface > (SRV_EPISODIC);
    EpisodicInterface srv_episodic;
    ServiceClient client_score = nodePtr->serviceClient < ScoreInterface > (SRV_SCORE);
    ScoreInterface srv_score;
    processPittInfo= true;
    ros::Rate loop_rate(10);


    while (ros::ok()) {
       // if(receivedNewShapes){

            //Calling the scene service
/*
            //Todo delate only for testing without pitt
            TrackedShape shapeSphere;
            shapeSphere.shape_tag=SPHERE;
            shapeSphere.x_pc_centroid=0.56;
            shapeSphere.y_pc_centroid=0.013;
            shapeSphere.z_pc_centroid=0.70;
            vector<float> coefficient;
            coefficient.push_back(0.56);
            coefficient.push_back(0.013);
            coefficient.push_back(0.70);
            coefficient.push_back(0.03);
            shapeSphere.coefficients=coefficient;
            shapeSphere.color.data="red";
            correctedRansacShapes.tracked_shapes.push_back(shapeSphere);
            TrackedShape shapeSphere1;
            shapeSphere1.shape_tag=CYLINDER;
            shapeSphere1.x_pc_centroid= 0.65;
            shapeSphere1.y_pc_centroid= -0.13;
            shapeSphere1.z_pc_centroid= 0.80;
            vector<float> coefficient1;
            coefficient1.push_back(0.815);
            coefficient1.push_back(-0.09);
            coefficient1.push_back(3.96);
            coefficient1.push_back(-0.04);
            coefficient1.push_back(-0.015);
            coefficient1.push_back(-0.99);
            coefficient1.push_back(0.015);

            shapeSphere1.coefficients=coefficient1;
            shapeSphere1.color.data="green";
            correctedRansacShapes.tracked_shapes.push_back(shapeSphere1);
            TrackedShape shapeSphere2;
            shapeSphere2.shape_tag=CONE;
            shapeSphere2.x_pc_centroid= 0.50;
            shapeSphere2.y_pc_centroid= -0.40;
            shapeSphere2.z_pc_centroid= 0.56;
            vector<float> coefficient2;
            coefficient2.push_back(-22.35);
            coefficient2.push_back(-63.05);
            coefficient2.push_back(-30.975);
            coefficient2.push_back(0.6506);
            coefficient2.push_back(0.553);
            coefficient2.push_back(0.52);
            coefficient2.push_back(0.46);
            shapeSphere2.coefficients=coefficient2;
            shapeSphere2.color.data="blue";
            correctedRansacShapes.tracked_shapes.push_back(shapeSphere2);
            receivedNewShapes=true;
            */

      int decision = userDecision();
        //memorization
        if(decision==1) {
            if (receivedNewShapes) {
                //calling the semantic service
                for (int i = 0; i < correctedRansacShapes.tracked_shapes.size(); i++) {
                    cout << "filling the srv request " << endl;
                    atom shapeSemanticService;
                    TrackedShape shapeRansac = correctedRansacShapes.tracked_shapes[i];
                    shapeSemanticService.type = shapeRansac.shape_tag;
                    shapeSemanticService.coefficients = coefficientsFromRansacToSemantic(shapeRansac);
                    shapeSemanticService.color = shapeRansac.color.data;
                    srv_semantic.request.geometricPrimitives.push_back(shapeSemanticService);
                }
                cout << "calling the service" << endl;
                if (client_semantic.call(srv_semantic)) {
                    cout << "called the service" << endl;
                    //calling the episodic service
                    srv_episodic.request.SceneName = srv_semantic.response.SceneName;
                    cout << "Scene Name \n" << srv_semantic.response.SceneName << endl;
                    srv_episodic.request.SubClasses = srv_semantic.response.SubClasses;
                    vector <string> SubC = srv_semantic.response.SubClasses;
                    cout << "SubClasses, Size : " << SubC.size();
                    for (int i = 0; i < SubC.size(); i++) {
                        cout << SubC[i] << endl;
                    }
                    srv_episodic.request.SuperClasses = srv_semantic.response.SuperClasses;
                    vector <string> SupC = srv_semantic.response.SuperClasses;
                    cout << "SuperClasses, Size : " << SupC.size();
                    for (int i = 0; i < SupC.size(); i++) {
                        cout << SupC[i] << endl;
                    }
                    /*
                    vector <string> FS = srv_semantic.response.FirstSuperClass;
                    cout << "first sup class, Size : " << FS.size();
                    for (int i = 0; i < FS.size(); i++) {
                        cout << FS[i] << endl;
                    }
                    vector <string> isFS = srv_semantic.response.isFirstSuperClassOf;
                    cout << "is first sup class, Size : " << isFS.size();
                    for (int i = 0; i < isFS.size(); i++) {
                        cout << isFS[i] << endl;
                    }
                    */
                    srv_episodic.request.Object = srv_semantic.response.Objects;
                    string support = "";
                    userCheck();
                    cout << "please insert the support Name" << endl;
                    getline(cin, support);
                    srv_episodic.request.SupportName = support;
                    if (client_episodic.call(srv_episodic)) {
                        cout << "episodic name" << srv_episodic.response.EpisodicSceneName << endl;
                        receivedNewShapes = false;
                    }
                    /*   if(srv_episodic.response.learnt || srv_semantic.response.learnt) {
                           if (srv_semantic.response.learnt) {
                               SemanticScoreItem semanticScoreItem;
                               semanticScoreItem.Name = srv_semantic.response.SceneName;
                               semanticScoreItem.FirstSuperClass = srv_semantic.response.FirstSuperClass;
                               semanticScoreItem.SubClasses = srv_semantic.response.SubClasses;
                               semanticScoreItem.SuperClasses = srv_semantic.response.SuperClasses;
                               semanticScoreItem.IsFirstSuperCLassOf = srv_semantic.response.isFirstSuperClassOf;
                               srv_score.request.Semantic = semanticScoreItem;
                           }
                           if (srv_episodic.response.learnt) {
                               EpisodicScoreItem episodicScoreItem;
                               episodicScoreItem.Name = srv_episodic.response.EpisodicSceneName;
                               cout << "Episodic Name \n" << srv_episodic.response.EpisodicSceneName << endl;
                               episodicScoreItem.NameSemanticItem = srv_semantic.response.SceneName;
                               srv_score.request.Episodic = episodicScoreItem;
                           }
                           if (client_score.call(srv_score)) {

                               receivedNewShapes = false;
                               processPittInfo = userCheckContinue("memorizing");

                           }
                       }
                       else{
                           processPittInfo=userCheckContinue("memorizing");
                       }

                   }
                   */


                }
            }
        }
        //retrieval
        else if(decision==2){
            int retrieval = userDecisionRetrieval();
            //semantic Retrieval
            if(retrieval==1){
                vector<RetrievalSemantic> retrievalSemantic;
                bool colorContinue = true;
                do {
                    RetrievalSemantic rs;
                    string color = "";
                    string label="";
                    int cardinality;
                    cout<<"insert color"<< endl;
                    getline(cin, color);
                    //MAP COLOR
                    rs.ObjectProperty= color;
                    cout<<"insert minimum cardinatlity"<<endl;
                    cin>>cardinality;
                    rs.minCardinality.data= cardinality;
                    cout<<"insert kind of primitive"<<endl;
                    getline(cin,label);
                    rs.Primitive=label;
                    retrievalSemantic.push_back(rs);
                    colorContinue=userCheckContinue("adding color information?");
                }
                while(colorContinue);
                bool srContinue=true;
                do {
                    RetrievalSemantic rs;
                    string spatialRelationship = "";
                    string label="";
                    int cardinality;
                    cout<<"insert spatialRelationship"<< endl;
                    getline(cin, spatialRelationship);
                    rs.ObjectProperty= spatialRelationship;
                    cout<<"insert minimum cardinatlity"<<endl;
                    cin>>cardinality;
                    rs.minCardinality.data= cardinality;
                    cout<<"insert kind of primitive"<<endl;
                    getline(cin,label);
                    rs.Primitive=label;
                    retrievalSemantic.push_back(rs);
                    srContinue=userCheckContinue("adding spatialRelationship?");
                }
                while(srContinue);
                srv_semantic.request.retrieval =retrievalSemantic;

            }
            //episodic Retrieval
            else if (retrieval==2){
                RetrievalEpisodic re;
                //support name
                string support= "";
                int timeInterval;
                cout<<"insert support name"<<endl ;
                getline(cin,support);
                re.support=support;
                //time interval
                cout<<"insert time interval"<<endl;
                cin>>timeInterval;
                re.time.data=timeInterval;
                //objectProperty
                vector<objectPropertyRetrieval> propertyRetrieval ;
                bool continueSpatialRelationship=true;
                do {
                    objectPropertyRetrieval objPropertyRetrieval;
                    retrievalAtom subject;
                    retrievalAtom object ;
                    //fill in the subject
                    string labelSubject ;
                    string colorSubject="";
                    string labelObject ;
                    string colorObject="";
                    string relationship="";
                    cout<<"insert tag of the subject"<<endl;
                    getline(cin,labelSubject);
                    subject.Label=labelSubject;
                    vector<geometricCharacteristic> geomCarSubject;
                    bool geometricCharacteristicContinue = true;
                    do{
                        geometricCharacteristic gc;
                        string feature = "";
                        int interval;
                        cout<<"inserti geometric feature"<<endl;
                        getline(cin,feature);
                        gc.characteristic=feature;
                        cout<<"insert interval"<<endl;
                        cin>>interval;
                        gc.interval= interval;
                        geomCarSubject.push_back(gc);
                        geometricCharacteristicContinue=userCheckContinue("insert geometric characteristics");
                    }while(geometricCharacteristicContinue);
                    subject.GeometricFeatures=geomCarSubject;
                    cout <<"insert color of primitive"<<endl;
                    getline(cin,colorSubject);
                    subject.color=colorSubject;
                    objPropertyRetrieval.subject=subject;
                    //spatial Relationship
                    cout<<"insert the relationship"<<endl ;
                    getline(cin,relationship);
                    objPropertyRetrieval.relationship=relationship;
                    //object
                    cout<<"insert tag of the object"<<endl;
                    getline(cin,labelObject);
                    object.Label=labelObject;
                    vector<geometricCharacteristic> geomCarObject;
                    do{
                        geometricCharacteristic gc;
                        string feature = "";
                        int interval;
                        cout<<"inserti geometric feature"<<endl;
                        getline(cin,feature);
                        gc.characteristic=feature;
                        cout<<"insert interval"<<endl;
                        cin>>interval;
                        gc.interval= interval;
                        geomCarObject.push_back(gc);
                        geometricCharacteristicContinue=userCheckContinue("insert geometric characteristics");
                    }while(geometricCharacteristicContinue);
                    object.GeometricFeatures=geomCarObject;
                    cout <<"insert color of primitive"<<endl;
                    getline(cin,colorObject);
                    object.color=colorObject;
                    objPropertyRetrieval.object=object;
                    propertyRetrieval.push_back(objPropertyRetrieval);
                    continueSpatialRelationship= userCheckContinue("adding spatial relationship");
                }while(continueSpatialRelationship);
                re.objectProperty=propertyRetrieval;
                vector<retrievalAtom> retrAtoms;
                bool retrievalAtomContinue= true ;
                do {
                    retrievalAtom atom ;
                    string label="";
                    string color ="";
                    cout<<"insert tag of the primitive"<<endl;
                    getline(cin,label);
                    atom.Label=label;
                    vector<geometricCharacteristic> geomCar;
                    bool geometricCharacteristicContinue=true;
                    do{
                        geometricCharacteristic gc;
                        string feature = "";
                        int interval;
                        cout<<"inserti geometric feature"<<endl;
                        getline(cin,feature);
                        gc.characteristic=feature;
                        cout<<"insert interval"<<endl;
                        cin>>interval;
                        gc.interval= interval;
                        geomCar.push_back(gc);
                        geometricCharacteristicContinue=userCheckContinue("insert geometric characteristics");
                    }while(geometricCharacteristicContinue);
                    atom.GeometricFeatures=geomCar;
                    cout <<"insert color of primitive"<<endl;
                    getline(cin,color);
                    atom.color=color;
                    retrAtoms.push_back(atom);
                    retrievalAtomContinue=userCheckContinue("adding primitives");
                }while(retrievalAtomContinue);
                re.primitives=retrAtoms;
                srv_episodic.request.retrieval=re;
            }

        }
        //forgetting

        else if(decision==3){

        }
        ros::spinOnce();
        loop_rate.sleep();

    }
    return 0;
}

vector <float> coefficientsFromRansacToSemantic(TrackedShape shape){
    vector <float> coefficientsRansac;
    coefficientsRansac.push_back(shape.x_pc_centroid);
    coefficientsRansac.push_back(shape.y_pc_centroid);
    coefficientsRansac.push_back(shape.z_pc_centroid);
    if(shape.shape_tag==SPHERE){
        coefficientsRansac.push_back(shape.coefficients[3]);
    }
    else if (shape.shape_tag==CYLINDER){
        coefficientsRansac.push_back(shape.coefficients[0]);
        coefficientsRansac.push_back(shape.coefficients[1]);
        coefficientsRansac.push_back(shape.coefficients[2]);
        coefficientsRansac.push_back(shape.coefficients[3]);
        coefficientsRansac.push_back(shape.coefficients[4]);
        coefficientsRansac.push_back(shape.coefficients[5]);
        coefficientsRansac.push_back(shape.coefficients[6]);
        //height of the cylinder since it is not given
        cout<<"insert value for height cyilinder"<<endl;
        float height;
        cin>>height;
        coefficientsRansac.push_back(height);
    }
    else if (shape.shape_tag==CONE){
       coefficientsRansac.push_back(shape.coefficients[0]);
        coefficientsRansac.push_back(shape.coefficients[1]);
        coefficientsRansac.push_back(shape.coefficients[2]);
        coefficientsRansac.push_back(shape.coefficients[3]);
        coefficientsRansac.push_back(shape.coefficients[4]);
        coefficientsRansac.push_back(shape.coefficients[5]);
        //radius
        cout<<"insert value for radius cone"<<endl;
        float radius;
        cin>>radius;
        coefficientsRansac.push_back(radius);
        //height
        cout<<"insert value for height cone"<<endl;
        float height;
        cin>>height;
        coefficientsRansac.push_back(height);
    }
    else if (shape.shape_tag==PLANE){
        coefficientsRansac.insert(coefficientsRansac.end(),shape.coefficients.begin(),shape.coefficients.end());
    }

    return coefficientsRansac;


}

