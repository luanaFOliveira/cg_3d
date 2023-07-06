#ifndef __BIELA_H__
#define __BIELA_H__

#include "gl_canvas2d.h"
#include "Cilindro.h"
#include "Paralelepipedo.h"
#include "Trapezio.h"

class Biela{

    Cilindro* circuloMenor;
    Cilindro* buracoCirculoMenor;
    Cilindro* circuloMaior;
    Cilindro* buracoCirculoMaior;
    Trapezio* braco;

    float xcCircMenor,ycCircMenor,xcCircMaior,ycCircMaior,raioMaior,raioMenor,tamBiela;


public:

    Biela(){

        xcCircMaior = 300;
        ycCircMaior = 640;
        xcCircMenor = xcCircMaior;
        ycCircMenor = ycCircMaior-200;
        raioMaior = 50;
        raioMenor = 20;
        tamBiela = 200;
        circuloMenor = new Cilindro(60,raioMenor,xcCircMenor,ycCircMenor,2);
        //buracoCirculoMenor = new Cilindro(60,raioMenor-10,xcCircMenor,ycCircMenor,2);
        circuloMaior = new Cilindro(60,raioMaior,xcCircMaior,ycCircMaior,2);
        //buracoCirculoMaior = new Cilindro(60,raioMaior-10,xcCircMaior,ycCircMaior,2);
        braco = new Trapezio(tamBiela,raioMenor,raioMaior,20,xcCircMenor,ycCircMenor+tamBiela/2,2);
    }


    void Render(){
        circuloMenor->Render();
        //buracoCirculoMenor->Render();
        circuloMaior->Render();
        //buracoCirculoMaior->Render();
        braco->Render();

    }

    void rotacionaBiela(int angulo){
        braco->rotateVec3d(angulo,8);

    }


    //getters
    float getXcentralCirculoMenor(){
        return xcCircMenor;
    }
    float getYcentralCirculoMenor(){
        return ycCircMenor;
    }
    float getXcentralCirculoMaior(){
        return xcCircMaior;
    }
    float getYcentralCirculoMaior(){
        return ycCircMaior;
    }
    float getRaioCirculoMenor(){
        return raioMenor;
    }
    float getRaioCirculoMaior(){
        return raioMaior;
    }
    float getTamanhoBiela(){
        return tamBiela;
    }

    //setters

    void setXcentralCirculoMenor(float newX){
        xcCircMenor = newX;
        //circuloMenor->setCenterX(newX);
        //braco->setCenterX(newX);
    }
    void setYcentralCirculoMenor(float newY){
        ycCircMenor = newY;
        //circuloMenor->setCenterY(newY);
        //braco->setCenterY(newY+tamBiela/2);

    }
    /*
    void newPointsBiela(float xc,float yc){
        float largPeq = raioMenor;
        float largGra = raioMaior;

        entrada[0].x = xc-largPeq;
        entrada[0].y = yc-altura/2;
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
*/



    void moveBiela(float x,float y){
        float centroXMaior = xcCircMaior+x;
        float centroYMaior = ycCircMaior+y+raioMaior;
        float yTrapezio = centroYMaior-tamBiela/2;
        ycCircMenor = centroYMaior-(tamBiela/sin(PI/4));
        xcCircMenor = ycCircMenor*cos(PI/4);
        circuloMaior->changePoints(60,raioMaior,centroXMaior,centroYMaior);
        circuloMenor->changePoints(60,raioMenor,xcCircMenor,ycCircMenor);

        float dx = xcCircMenor - centroXMaior;
        float dy = ycCircMenor - yTrapezio;
        float angBi = atan2(dy,dx);
        float angBiGraus = angBi*180/PI;
        braco->changePoints(raioMenor,raioMaior,xcCircMenor,yTrapezio);
        braco->rotateVec3d(angBiGraus,8);

    }

    void moveBracoBiela(float x,float y){
        //braco->changePoints(raioMenor,raioMaior,x,y);
    }

    void moveCirculoMaior(float newX,float newY){
        xcCircMaior = newX;
        ycCircMaior = newY;
        ycCircMenor = newY - tamBiela/2;
        circuloMaior->changePoints(60,raioMaior,newX,newY);
        circuloMenor->changePoints(60,raioMenor,xcCircMenor,newY-tamBiela/2);
        //braco->changePoints(raioMenor,raioMaior,xcCircMenor,ycCircMenor+tamBiela/2);

    }

    void moveCirculoMenor(float newX,float newY){
        ycCircMenor = newY - tamBiela/2;
        circuloMenor->changePoints(60,raioMenor,xcCircMenor,newY-tamBiela/2+300);
    }

    void setXcentralCirculoMaior(float newX,float centerX){
        xcCircMaior = newX;
        circuloMaior->setCenterX(xcCircMaior);

    }
    void setYcentralCirculoMaior(float newY,float centerY){
        ycCircMaior = newY;
        circuloMaior->setCenterY(ycCircMaior);
    }

    void setRaioCirculoMenor(float newRaio){
        raioMenor = newRaio;
    }
    void setRaioCirculoMaior(float newRaio){
        raioMaior = newRaio;
    }
    void setTamanhoBiela(float newTam){
        tamBiela = newTam;
    }


};

#endif




