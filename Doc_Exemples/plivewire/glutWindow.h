////////////////////////////////////////////////////////////////
//                                                            //
// glutWindow.h                                               //
// beta version 0.3 - 9/9/97)                                 //
//                                                            //
// George Stetten and Korin Crawford                          //
// copyright given to the public domain                       //
//                                                            //
// Please email comments to stetten@acpub.duke.edu,           //
//                                                            //
////////////////////////////////////////////////////////////////


#ifndef __GLUT_WINDOW_H__
#define __GLUT_WINDOW_H__

class GlutWindow{
protected:

   int          windowID;

public:
   int width, height;

   GlutWindow(void);
   ~GlutWindow();

   virtual void CallBackDisplayFunc();
   virtual void CallBackMenuFunc(int value);
   virtual void CallBackIdleFunc(void);
   virtual void CallBackKeyboardFunc(unsigned char key, int x, int y);
   virtual void CallBackMotionFunc(int x, int y);
   virtual void CallBackMouseFunc(int button, int state, int x, int y);
   virtual void CallBackPassiveMotionFunc(int x, int y);
   virtual void CallBackReshapeFunc(int w, int h);   
   virtual void CallBackSpecialFunc(int key, int x, int y);   
   virtual void CallBackVisibilityFunc(int visible);

   void    SetWindowID(int newWindowID);
   int     GetWindowID(void);

};

#endif






