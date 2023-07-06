#ifndef __PISTAO_H__
#define __PISTAO_H__

#include "gl_canvas2d.h"
#include "Cilindro.h"


class Pistao{
    Cilindro* cilindroPrincipal;
    float alt,raio,translateX,translateY;

public:

    Pistao(){
        alt = 40;
        raio = 50;
        translateX = 300;
        translateY = 430-raio;
        cilindroPrincipal = new Cilindro(alt,raio,translateX,translateY,2);
    }


    void Render(){
        cilindroPrincipal->Render();
    }

    void movePistao(float y){
        cilindroPrincipal->changePoints(alt,raio,translateX,y-raio);
    }

};

#endif




