#ifndef __PARALELEPIPEDO_H__
#define __PARALELEPIPEDO_H__

#include "gl_canvas2d.h"
#include "Vector3.h"

// typeRotate == 1 ->rotate X
// typeRotate == 2 ->rotate Y
// typeRotate == 3 ->rotate Z


class Paralelepipedo{
    Vector3 entrada[8];
    float altura,largura,profundidade;
    float angle = 0.0;
    float d = 151;
    float translateX,translateY;
    int typeRotate;


public:

    Paralelepipedo(float alt,float larg,float prof,float translateX_,float translateY_,int typeRotate_){
        altura = alt;
        largura = larg;
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

        entrada[0].x = -largura/2;
        entrada[0].y = -altura/2;
        entrada[0].z = -profundidade/2;

        entrada[1].x = largura/2;
        entrada[1].y = -altura/2;
        entrada[1].z = -profundidade/2;

        entrada[2].x = largura/2;
        entrada[2].y = altura/2;
        entrada[2].z = -profundidade/2;

        entrada[3].x = -largura/2;
        entrada[3].y = altura/2;
        entrada[3].z = -profundidade/2;

        entrada[4].x = -largura/2;
        entrada[4].y = -altura/2;
        entrada[4].z = profundidade/2;

        entrada[5].x = largura/2;
        entrada[5].y = -altura/2;
        entrada[5].z = profundidade/2;

        entrada[6].x = largura/2;
        entrada[6].y = altura/2;
        entrada[6].z = profundidade/2;

        entrada[7].x = -largura/2;
        entrada[7].y = altura/2;
        entrada[7].z = profundidade/2;

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
                p = rotatePointAroundXAxis(p, angle);
            }else if(typeRotate == 2){
                p = rotatePointAroundYAxis(p, angle);
            }else if(typeRotate == 3){
                p = rotatePointAroundZAxis(p, angle);
            }
            saida[i] = project(p, d);
        }

        CV::translate(translateX, translateY);
        CV::color(0);

        angle += 0.3;
        desenhaParalelepipedo(saida);

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
            {0, 1, 0},
            {cos(theta), 0, sin(theta)},
            {-sin(theta), 0, cos(theta)}
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
            {cos(theta), 0, sin(theta)},
            {-sin(theta), 0, cos(theta)},
            {0, 1, 0},
        };

        // Realiza a multiplicação da matriz de rotação pelo ponto
        rotatedPoint.x = rotationMatrix[0][0] * p.x + rotationMatrix[0][1] * p.y + rotationMatrix[0][2] * p.z;
        rotatedPoint.y = rotationMatrix[1][0] * p.x + rotationMatrix[1][1] * p.y + rotationMatrix[1][2] * p.z;
        rotatedPoint.z = rotationMatrix[2][0] * p.x + rotationMatrix[2][1] * p.y + rotationMatrix[2][2] * p.z;

        return rotatedPoint;
    }




};

#endif




