function varargout = polarization_controller_GUI(varargin)
% POLARIZATION_CONTROLLER_GUI MATLAB code for polarization_controller_GUI.fig
%      POLARIZATION_CONTROLLER_GUI, by itself, creates a new POLARIZATION_CONTROLLER_GUI or raises the existing
%      singleton*.
%
%      H = POLARIZATION_CONTROLLER_GUI returns the handle to a new POLARIZATION_CONTROLLER_GUI or the handle to
%      the existing singleton*.
%
%      POLARIZATION_CONTROLLER_GUI('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in POLARIZATION_CONTROLLER_GUI.M with the given input arguments.
%
%      POLARIZATION_CONTROLLER_GUI('Property','Value',...) creates a new POLARIZATION_CONTROLLER_GUI or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before polarization_controller_GUI_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to polarization_controller_GUI_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES
% Edit the above text to modify the response to help polarization_controller_GUI
% Last Modified by GUIDE v2.5 26-Sep-2020 20:44:18
% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @polarization_controller_GUI_OpeningFcn, ...
                   'gui_OutputFcn',  @polarization_controller_GUI_OutputFcn, ...
                   'gui_LayoutFcn',  [] , ...
                   'gui_Callback',   []);
if nargin && ischar(varargin{1})
    gui_State.gui_Callback = str2func(varargin{1});
end

if nargout
    [varargout{1:nargout}] = gui_mainfcn(gui_State, varargin{:});
else
    gui_mainfcn(gui_State, varargin{:});
end
% End initialization code - DO NOT EDIT

% --- Executes just before GUI is made visible.
function polarization_controller_GUI_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to polarization_controller_GUI (see VARARGIN)

% Choose default command line output for polarization_controller_GUI
handles.output = hObject;
% Update handles structure
guidata(hObject, handles);
% UIWAIT makes polarization_controller_GUI wait for user response (see UIRESUME)
% uiwait(handles.figure1);
clc;
clear;

% --- Outputs from this function are returned to the command line.
function varargout = polarization_controller_GUI_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% Get default command line output from handles structure
varargout{1} = handles.output;

% --- Executes during object creation, after setting all properties.
function slider1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end

% --- Executes during object creation, after setting all properties.
function slider3_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider3 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end



function wave(handles_slider1,handles_slider2,handles_slider3,handles_txt1,handles_txt2,handles_txt3,handles_txts0,handles_txts1,handles_txts2,handles_txts3,handles_txts01,handles_txts11,handles_txts21,handles_txts31)

c = 10; %speed of EM wave
lambda = 1; %wave length
z = 0; %z=0 mesure from a point with time
T = lambda/c; % c = Lambda/T -> T = Lambda/c
t = 0:T/100:T;% let time flow
%Amplitudes
Ex = 1;
Ey = 1;
% px - py = phase difference
px = 0;
py = 0;%T/4;

omega = 2*pi*c/lambda; %2pi/T
k = 2*pi/lambda;

ex = real(Ex*exp( 1i*(omega*(t + px) - k*z ) ));
ey = real(Ey*exp( 1i*(omega*(t + py) - k*z ) ));

%Jones vector
J1 = Ex*exp(1i*omega*px);
J2 = Ey*exp(1i*omega*py);

J = [ J1
      J2 ];

%normalized
factor = (abs(J1)^2 + abs(J2)^2)^(0.5);
J = J/factor; %normalized jones vector

[Theta_x, eox] = cart2pol( real(J(1)), imag(J(1)) );
[Theta_y, eoy] = cart2pol( real(J(2)), imag(J(2)) );
delta = Theta_y - Theta_x;

s0 = (eox^2) + (eoy^2);
s1 = (eox^2) - (eoy^2);
s2 = 2*eox*eoy*cos(delta);
s3 = 2*eox*eoy*sin(delta);

if s1 < 1e-10 && s1 > -1e-10
    s1 = 0;
end
if s2 < 1e-10 && s2 > -1e-10
    s2 = 0;
end
if s3 < 1e-10 && s3 > -1e-10
    s3 = 0;
end

set(handles_txts01, 'String',strcat('S0','__',num2str(s0)));
set(handles_txts11, 'String',strcat('S1','__',num2str(s1)));
set(handles_txts21, 'String',strcat('S2','__',num2str(s2)));
set(handles_txts31, 'String',strcat('S3','__',num2str(s3)));

p_theta_w4_1 = get(handles_slider1,'Value');
p_theta_w2_2 = get(handles_slider2,'Value');
p_theta_w4_3 = get(handles_slider3,'Value');

set(handles_txt1, 'String', num2str(p_theta_w4_1));
set(handles_txt2, 'String', num2str(p_theta_w2_2));
set(handles_txt3, 'String', num2str(p_theta_w4_3));

p_rad1 = (pi*p_theta_w4_1)/180;
p_rad2 = (pi*p_theta_w2_2)/180;
p_rad3 = (pi*p_theta_w4_3)/180;


polarizer_w4_1 = [ cos(p_rad1)^2 + 1i*(sin(p_rad1)^2)  (1-1i)*sin(p_rad1)*cos(p_rad1)
                   (1-1i)*sin(p_rad1)*cos(p_rad1)      sin(p_rad1)^2 + 1i*(cos(p_rad1)^2) ];
polarizer_w4_1 = polarizer_w4_1*exp(-1i*pi/4);

polarizer_w2_2 = [ cos(p_rad2)^2 - sin(p_rad2)^2  2*sin(p_rad2)*cos(p_rad2)
                   2*sin(p_rad2)*cos(p_rad2)      sin(p_rad2)^2 - cos(p_rad2)^2 ];
polarizer_w2_2 = polarizer_w2_2*exp(-1i*pi/2);

polarizer_w4_3 = [ cos(p_rad3)^2 + 1i*(sin(p_rad3)^2)  (1-1i)*sin(p_rad3)*cos(p_rad3)
                   (1-1i)*sin(p_rad3)*cos(p_rad3)      sin(p_rad3)^2 + 1i*(cos(p_rad3)^2) ];
polarizer_w4_3 = polarizer_w4_3*exp(-1i*pi/4);

output = polarizer_w4_3*polarizer_w2_2*polarizer_w4_1*J;

ex_o = real(exp( 1i*(omega*t - k*z ))*output(1) );
ey_o = real(exp( 1i*(omega*t - k*z ))*output(2) );

[Theta_x, eox] = cart2pol( real(output(1)), imag(output(1)) );
[Theta_y, eoy] = cart2pol( real(output(2)), imag(output(2)) );
delta = Theta_y - Theta_x;

s0 = (eox^2) + (eoy^2);
s1 = (eox^2) - (eoy^2);
s2 = 2*eox*eoy*cos(delta);
s3 = 2*eox*eoy*sin(delta);

if s1 < 1e-10 && s1 > -1e-10
    s1 = 0;
end
if s2 < 1e-10 && s2 > -1e-10
    s2 = 0;
end
if s3 < 1e-10 && s3 > -1e-10
    s3 = 0;
end

set(handles_txts0, 'String',strcat('S0','__',num2str(s0)));
set(handles_txts1, 'String',strcat('S1','__',num2str(s1)));
set(handles_txts2, 'String',strcat('S2','__',num2str(s2)));
set(handles_txts3, 'String',strcat('S3','__',num2str(s3)));

plot3(t,ex_o,t*0,'r',t,t*0,ey_o,'b',t,ex_o,ey_o,'g','LineWidth',2),axis([-1,1,-1,1,-1,1]),grid on,view(90,0);
%figure(2),plot3(t,ex,ey,'g','LineWidth',2),grid on,view(90,0);


% --- Executes on slider movement.
function slider1_Callback(hObject, eventdata, handles)

wave(handles.slider1,handles.slider2,handles.slider3,handles.txt1,handles.txt2,handles.txt3,handles.txts0,handles.txts1,handles.txts2,handles.txts3,handles.txts01,handles.txts11,handles.txts21,handles.txts31);
% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)

wave(handles.slider1,handles.slider2,handles.slider3,handles.txt1,handles.txt2,handles.txt3,handles.txts0,handles.txts1,handles.txts2,handles.txts3,handles.txts01,handles.txts11,handles.txts21,handles.txts31);
% --- Executes on slider movement.
function slider3_Callback(hObject, eventdata, handles)

wave(handles.slider1,handles.slider2,handles.slider3,handles.txt1,handles.txt2,handles.txt3,handles.txts0,handles.txts1,handles.txts2,handles.txts3,handles.txts01,handles.txts11,handles.txts21,handles.txts31);
% --- Executes on slider movement.
