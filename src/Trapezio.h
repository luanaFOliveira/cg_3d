#ifndef __TRAPEZIO_H__
#define __TRAPEZIO_H__

#include "gl_canvas2d.h"
#include "Vector3.h"

// typeRotate == 1 ->rotate X
// typeRotate == 2 ->rotate Y
// typeRotate == 3 ->rotate Z


class Trapezio{
    Vector3 entrada[8];
    float altura,largPeq,largGra,profundidade,x1,x2,y1,y2;
    float angle = 0.0;
    float d = 151;
    float translateX,translateY;
    int typeRotate;


public:

    Trapezio(float alt,float largPequena,float largGrande,float prof,float translateX_,float translateY_,int typeRotate_){
        altura = alt;
        largPeq = largPequena;
        largGra = largGrande;
        profundidade = prof;
        translateX = translateX_;
        translateY = translateY_;
        typeRotate = typeRotate_;
        createPoints();

    }

    void createPoints(){
        for (int i = 0; i < 8; i++) {
            entrada[i] = *new Vector3();
        }


        entrada[0].x = -largPeq;
        entrada[0].y = -altura/2;
        entrada[0].z = -profundidade/2;

        entrada[1].x = largPeq;
        entrada[1].y = -altura/2;
        entrada[1].z = -profundidade/2;

        entrada[2].x = largGra;
        entrada[2].y = altura/2;
        entrada[2].z = -profundidade/2;

        entrada[3].x = -largGra;
        entrada[3].y = altura/2;
        entrada[3].z = -profundidade/2;

        entrada[4].x = -largPeq;
        entrada[4].y = -altura/2;
        entrada[4].z = profundidade/2;

        entrada[5].x = largPeq;
        entrada[5].y = -altura/2;
        entrada[5].z = profundidade/2;

        entrada[6].x = largGra;
        entrada[6].y = altura/2;
        entrada[6].z = profundidade/2;

        entrada[7].x = -largGra;
        entrada[7].y = altura/2;
        entrada[7].z = profundidade/2;

    }




    void changePoints(float largPequeno,float largGrande,float transX,float transY){
        translateX = transX;
        translateY = transY;
        largPeq = largPequeno;
        largGra = largGrande;

        entrada[0].x = -largPeq;
        entrada[0].y = -altura/2;
        entrada[0].z = -profundidade/2;

        entrada[1].x = largPeq;
        entrada[1].y = -altura/2;
        entrada[1].z = -profundidade/2;

        entrada[2].x = largGra;
        entrada[2].y = altura/2;
        entrada[2].z = -profundidade/2;

        entrada[3].x = -largGra;
        entrada[3].y = altura/2;
        entrada[3].z = -profundidade/2;

        entrada[4].x = -largPeq;
        entrada[4].y = -altura/2;
        entrada[4].z = profundidade/2;

        entrada[5].x = largPeq;
        entrada[5].y = -altura/2;
        entrada[5].z = profundidade/2;

        entrada[6].x = largGra;
        entrada[6].y = altura/2;
        entrada[6].z = profundidade/2;

        entrada[7].x = -largGra;
        entrada[7].y = altura/2;
        entrada[7].z = profundidade/2;


    }

    void setPoints(Vector3 trap[]){

        entrada[0].x = trap[0].x;
        entrada[0].y = trap[0].y;
        entrada[0].z = trap[0].z;

        entrada[1].x = trap[1].x;
        entrada[1].y = trap[1].y;
        entrada[1].z = trap[1].z;

        entrada[2].x = trap[2].x;
        entrada[2].y = trap[2].y;
        entrada[2].z = trap[2].z;

        entrada[3].x = trap[3].x;
        entrada[3].y = trap[3].y;
        entrada[3].z = trap[3].z;

        entrada[4].x = trap[4].x;
        entrada[4].y = trap[4].y;
        entrada[4].z = trap[4].z;

        entrada[5].x = trap[5].x;
        entrada[5].y = trap[5].y;
        entrada[5].z = trap[5].z;

        entrada[6].x = trap[6].x;
        entrada[6].y = trap[6].y;
        entrada[6].z = trap[6].z;

        entrada[7].x = trap[7].x;
        entrada[7].y = trap[7].y;
        entrada[7].z = trap[7].z;

    }

    Vector3 getPoint(int index){
        return entrada[index];
    }

    void setCenterX(float newCenter){
        translateX = newCenter;
    }

    void setCenterY(float newCenter){
        translateY = newCenter;
    }

    float getCenterX(){
        return translateX;
    }

    float getCenterY(){
        return translateY;
    }


    float getPointXCentroLadoMenor(){
        return (entrada[0].x+entrada[1].x)/2.0;
    }

    float getPointYCentroLadoMenor(){
        return (entrada[0].y+entrada[1].y)/2.0;
    }

    void desenhaParalelepipedo(Vector2 para[8]) {
       int secCount = 5;
       for (int i = 1; i < 4; i++) {
          //quadrado da frente
          CV::line(para[i-1].x, para[i-1].y, para[i].x, para[i].y);
          //quadrado do fundo
          CV::line(para[secCount - 1].x, para[secCount - 1].y, para[secCount].x, para[secCount].y);
          secCount++;
       }
       CV::line(para[3].x, para[3].y, para[0].x, para[0].y);
       CV::line(para[7].x, para[7].y, para[4].x, para[4].y);

       // arestas faltantes
       for (int i = 0; i < 4; i++) {
          CV::line(para[i].x, para[i].y, para[i+4].x, para[i+4].y);
       }
    }


    void Render(){

        Vector3 p;
        Vector2 saida[8];

        for (int i = 0; i < 8; i++) {
            p = entrada[i];
            if(typeRotate == 1){
                //p = rotatePointAroundXAxis(p, angle);
            }else if(typeRotate == 2){
                //p = rotatePointAroundYAxis(p, angle);
            }else if(typeRotate == 3){
                //p = rotatePointAroundZAxis(p, angle);
            }
            saida[i] = project(p, d);
        }

        CV::translate(translateX, translateY);
        CV::color(0);

        angle += 0.3;
        desenhaParalelepipedo(saida);

    }



    void rotateVec3d(float angulo, int n) {

        double theta = angulo * PI / 180.0;
        float cx = calculaCentroX(entrada,n);
        float cy = calculaCentroY(entrada,n);
        float cz = calculaCentroZ(entrada,n);

        float c = cos(theta);
        float s = sin(theta);
        float rotationMatrix[3][3] = {
            {c, -s, 0},
            {s, c, 0},
            {0, 0, 1}
        };

        for (int i = 0; i < n; i++) {
            float x = entrada[i].x - cx;
            float y = entrada[i].y - cy;
            float z = entrada[i].z - cz;

            float new_x = rotationMatrix[0][0] * x + rotationMatrix[0][1] * y + rotationMatrix[0][2] * z;
            float new_y = rotationMatrix[1][0] * x + rotationMatrix[1][1] * y + rotationMatrix[1][2] * z;
            float new_z = rotationMatrix[2][0] * x + rotationMatrix[2][1] * y + rotationMatrix[2][2] * z;

            //vetor[i].set(new_x + cx,new_y + cy,new_z + cz);
            entrada[i].x = new_x + cx;
            entrada[i].y = new_y + cy;
            entrada[i].z = new_z + cz;



        }
    }


private:

    Vector2 project(Vector3 v, float d) {
       Vector2 v2;
       v2.x = v.x * d / (v.z + d);
       v2.y = v.y * d / (v.z + d);

       return v2;
    }


 Vector3 rotatePointAroundYAxis(Vector3 p, double angle) {
        Vector3 rotatedPoint;

        // Conversão do ângulo de graus para radianos
        double theta = angle * PI / 180.0;

        // Matriz de rotação em torno do eixo y
        double rotationMatrix[3][3] = {
            {cos(theta), 0, sin(theta)},
            {0, 1, 0},
            {-sin(theta), 0, cos(theta)}
        };

        // Realiza a multiplicação da matriz de rotação pelo ponto
        rotatedPoint.x = rotationMatrix[0][0] * p.x + rotationMatrix[0][1] * p.y + rotationMatrix[0][2] * p.z;
        rotatedPoint.y = rotationMatrix[1][0] * p.x + rotationMatrix[1][1] * p.y + rotationMatrix[1][2] * p.z;
        rotatedPoint.z = rotationMatrix[2][0] * p.x + rotationMatrix[2][1] * p.y + rotationMatrix[2][2] * p.z;

        return rotatedPoint;
    }



    Vector3 rotatePointAroundXAxis(Vector3 p, double angle) {
        Vector3 rotatedPoint;

        // Conversão do ângulo de graus para radianos
        double theta = angle * PI / 180.0;

        // Matriz de rotação em torno do eixo y
        double rotationMatrix[3][3] = {
            {1, 0, 0},
            {0,cos(theta),sin(theta)},
            {0,-sin(theta),cos(theta)}
        };

        // Realiza a multiplicação da matriz de rotação pelo ponto
        rotatedPoint.x = rotationMatrix[0][0] * p.x + rotationMatrix[0][1] * p.y + rotationMatrix[0][2] * p.z;
        rotatedPoint.y = rotationMatrix[1][0] * p.x + rotationMatrix[1][1] * p.y + rotationMatrix[1][2] * p.z;
        rotatedPoint.z = rotationMatrix[2][0] * p.x + rotationMatrix[2][1] * p.y + rotationMatrix[2][2] * p.z;

        return rotatedPoint;
    }


    Vector3 rotatePointAroundZAxis(Vector3 p, double angle) {
        Vector3 rotatedPoint;

        // Conversão do ângulo de graus para radianos
        double theta = angle * PI / 180.0;

        // Matriz de rotação em torno do eixo y
        double rotationMatrix[3][3] = {
            {cos(theta),sin(theta),0},
            {-sin(theta),cos(theta),0},
            {0, 0, 1},
        };

        // Realiza a multiplicação da matriz de rotação pelo ponto
        rotatedPoint.x = rotationMatrix[0][0] * p.x + rotationMatrix[0][1] * p.y + rotationMatrix[0][2] * p.z;
        rotatedPoint.y = rotationMatrix[1][0] * p.x + rotationMatrix[1][1] * p.y + rotationMatrix[1][2] * p.z;
        rotatedPoint.z = rotationMatrix[2][0] * p.x + rotationMatrix[2][1] * p.y + rotationMatrix[2][2] * p.z;

        return rotatedPoint;
    }


    float calculaCentroX(Vector3 *vetor,int n){
        float sumx=0;
        for(int i =0;i<n;i++){
            sumx += vetor[i].x;

        }

        return sumx/n;
    }


    float calculaCentroY(Vector3 *vetor,int n){
        float sumx=0;
        for(int i =0;i<n;i++){
            sumx += vetor[i].y;

        }

        return sumx/n;
    }


    float calculaCentroZ(Vector3 *vetor,int n){
        float sumx=0;
        for(int i =0;i<n;i++){
            sumx += vetor[i].z;

        }

        return sumx/n;
    }




};

#endif





