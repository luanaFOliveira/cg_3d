#ifndef __CILINDRO_H__
#define __CILINDRO_H__

#include "gl_canvas2d.h"
#include "Vector3.h"

// typeRotate == 1 ->rotate X
// typeRotate == 2 ->rotate Y
// typeRotate == 3 ->rotate Z


class Cilindro{
    Vector3 pontosConectamRetas[32];
    //Vector3 circunferenciaCima[36000];
    //Vector3 circunferenciaBaixo[36000];
    int segments = 16;
    float raio;
    float altura;
    float angle = 0.0;
    float d = 131;
    float translateX,translateY;
    int typeRotate;


public:

    Cilindro(float alt,float raio_,float translateX_,float translateY_,int typeRotate_){
        altura = alt;
        raio = raio_;
        translateX = translateX_;
        translateY = translateY_;
        typeRotate = typeRotate_;
        for (int i = 0; i < 32; i++) {
            pontosConectamRetas[i] = *new Vector3();
        }
        createPoints();

    }

    void changePoints(float alt,float raio_,float translateX_,float translateY_){
        altura = alt;
        raio = raio_;
        translateX = translateX_;
        translateY = translateY_;
        createPoints();
    }

    void createPoints(){


        for (int i = 0; i < segments; i++) {
          float ang = (2 * PI * i) / segments;  // ângulo do ponto atual

          // ponto que conectam da base inferior
          pontosConectamRetas[i].x = cos(ang) * raio;
          pontosConectamRetas[i].y = sin(ang) * raio;
          pontosConectamRetas[i].z = -altura / 2;

          // ponto que conectam da base superior
          pontosConectamRetas[i + segments].x = cos(ang) * raio;
          pontosConectamRetas[i + segments].y = sin(ang) * raio;
          pontosConectamRetas[i + segments].z = altura / 2;
        }

    }



    void desenhaCilindro(Vector2 cylinder[]) {
    // desenhar as linhas laterais do cilindro
        for (int i = 0; i < segments; i++) {
            int nextIndex = (i + 1) % segments;

            // linha da base inferior
            CV::line(cylinder[i].x, cylinder[i].y, cylinder[nextIndex].x, cylinder[nextIndex].y);

            // linha da base superior
            CV::line(cylinder[i + segments].x, cylinder[i + segments].y, cylinder[nextIndex + segments].x, cylinder[nextIndex + segments].y);

            // linha vertical que conecta as bases
            CV::line(cylinder[i].x, cylinder[i].y, cylinder[i + segments].x, cylinder[i + segments].y);
        }
        for (int i = 0; i < segments; i++) {
            // Base inferior
            CV::point(cylinder[i].x, cylinder[i].y);

            // Base superior
            CV::point(cylinder[i + segments].x, cylinder[i + segments].y);
        }

        // conectar o último ponto com o primeiro ponto para fechar as bases
        CV::line(cylinder[segments - 1].x, cylinder[segments - 1].y, cylinder[0].x, cylinder[0].y);
        CV::line(cylinder[segments - 1 + segments].x, cylinder[segments - 1 + segments].y, cylinder[segments].x, cylinder[segments].y);

    }

    void setCenterX(float newCenter){
        translateX = newCenter;
    }

    void setCenterY(float newCenter){
        translateY = newCenter;
    }

    void transaladar(float newX,float newY){
        CV::translate(newX,newY);
        translateX = newX;
        translateY = newY;
    }

    float getCenterX(){
        return translateX;
    }

    float getCenterY(){
        return translateY;
    }


    void Render(){
        Vector3 p;
        Vector2 saidaCilindro[32];

        for (int i = 0; i < 32; i++) {
            p = pontosConectamRetas[i];

            if(typeRotate == 1){
                //p = rotatePointAroundXAxis(p, angle);
            }else if(typeRotate == 2){
               // p = rotatePointAroundYAxis(p, angle);
            }else if(typeRotate == 3){
                //p = rotatePointAroundZAxis(p, angle);

            }
            saidaCilindro[i] = project(p, d);
        }

        CV::translate(translateX, translateY);
        CV::color(0);

        angle += 0.3;
        desenhaCilindro(saidaCilindro);
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




};

#endif



