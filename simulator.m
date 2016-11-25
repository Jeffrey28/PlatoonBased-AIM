function varargout = simulator(varargin)
% SIMULATOR MATLAB code for simulator.fig
%% 
%% 
%      SIMULATOR, by itself, creates a new SIMULATOR or raises the existing
%      singleton*.
%
%      H = SIMULATOR returns the handle to a new SIMULATOR or the handle to
%      the existing singleton*.
%
%      SIMULATOR('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SIMULATOR.M with the given input arguments.
%
%      SIMULATOR('Property','Value',...) creates a new SIMULATOR or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before simulator_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to simulator_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help simulator

% Last Modified by GUIDE v2.5 26-Sep-2016 15:09:27

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @simulator_OpeningFcn, ...
                   'gui_OutputFcn',  @simulator_OutputFcn, ...
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


% --- Executes just before simulator is made visible.
function simulator_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to simulator (see VARARGIN)

% Choose default command line output for simulator
handles.output = hObject;

% Update handles structure
handles.simTime = 500;
handles.readyColor = [1 1 1];
handles.runningColor = [104/255 151/255 187/255];
handles.finishedColor = [102/255 205/255 170/255];
handles.savingColor = [211/255 71/255 31/255];
grid on;
grid minor;
guidata(hObject, handles);
axes(handles.axes1);
%title('Platoon-Based Autonomous Intersection Management');
xlim([0 400]);
ylim([0 400]);
%grid on
%grid minor
set(handles.resetbutton,'Enable','off');
set(handles.savevideo,'Enable','off');

%axes(handles.axes1);




% UIWAIT makes simulator wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = simulator_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in start.
function start_Callback(hObject, eventdata, handles)
% hObject    handle to start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%clf;
handles.simTime = round(get(handles.duration,'Value'));

axes(handles.axes1);
contents = str2double(get(handles.granularity,'String'));
g = contents(get(handles.granularity,'Value'));
contents = str2double(get(handles.platoonSize,'String'));
maxSize= contents(get(handles.platoonSize,'Value'));
set(handles.resetbutton,'Enable','on');
set(handles.savevideo,'Enable','off');

set(handles.start,'Enable','off');

set(handles.status, 'BackgroundColor',handles.runningColor);
set(handles.status, 'String','Simulation is Running');
handles.F = AIM(g,maxSize,1,handles.simTime);
guidata(hObject,handles);
set(handles.status, 'BackgroundColor',handles.finishedColor);
set(handles.status, 'String','Finished');
set(handles.start,'Enable','on');
set(handles.resetbutton,'Enable','off');
set(handles.savevideo,'Enable','on');





% --- Executes on selection change in granularity.
function granularity_Callback(hObject, eventdata, handles)
% hObject    handle to granularity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns granularity contents as cell array
%        contents{get(hObject,'Value')} returns selected item from granularity


% --- Executes during object creation, after setting all properties.
function granularity_CreateFcn(hObject, eventdata, handles)
% hObject    handle to granularity (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in resetbutton.
function resetbutton_Callback(hObject, eventdata, handles)
% hObject    handle to resetbutton (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
%axes(handles.axes1);
%error('stop requested by user');
%handles.stop=1;
%guidata(hObject,handles);
%clc;
cla;
grid on;
grid minor;
set(handles.status, 'BackgroundColor',handles.finishedColor);
set(handles.status, 'String','Ready!');
set(handles.start,'Enable','on');
set(handles.savevideo,'Enable','off');



% --- Executes on slider movement.
function duration_Callback(hObject, eventdata, handles)
% hObject    handle to duration (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider
handles.simTime = round(get(hObject,'Value'));
labelText = strcat(num2str(handles.simTime),' Sec.');
set(handles.durationlabel,'String',labelText);
guidata(hObject,handles);


% --- Executes during object creation, after setting all properties.
function duration_CreateFcn(hObject, eventdata, handles)
% hObject    handle to duration (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on slider movement.
function slider2_Callback(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider2_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider2 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on button press in exit.
function exit_Callback(hObject, eventdata, handles)
% hObject    handle to exit (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
close(gcf);


% --- If Enable == 'on', executes on mouse press in 5 pixel border.
% --- Otherwise, executes on mouse press in 5 pixel border or over start.
function start_ButtonDownFcn(hObject, eventdata, handles)
% hObject    handle to start (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in savevideo.
function savevideo_Callback(hObject, eventdata, handles)
% hObject    handle to savevideo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
set(handles.start,'Enable','off');
set(handles.status, 'BackgroundColor',handles.savingColor);
set(handles.status, 'String','Saving Video');
pause(0.5);
video = VideoWriter('AIM-Demo.mp4','MPEG-4');
video.FrameRate = 60;
open(video);
writeVideo(video,handles.F);
close(video);
set(handles.status, 'BackgroundColor',handles.finishedColor);
set(handles.status, 'String','Video was saved to File!');
set(handles.start,'Enable','on');


% --- Executes on slider movement.
function slider5_Callback(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider


% --- Executes during object creation, after setting all properties.
function slider5_CreateFcn(hObject, eventdata, handles)
% hObject    handle to slider5 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes on selection change in platoonSize.
function platoonSize_Callback(hObject, eventdata, handles)
% hObject    handle to platoonSize (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns platoonSize contents as cell array
%        contents{get(hObject,'Value')} returns selected item from platoonSize


% --- Executes during object creation, after setting all properties.
function platoonSize_CreateFcn(hObject, eventdata, handles)
% hObject    handle to platoonSize (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: popupmenu controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end
