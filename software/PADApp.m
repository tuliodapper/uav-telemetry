function varargout = PADApp(varargin)
% PADAPP MATLAB code for PADApp.fig
%      PADAPP, by itself, creates a new PADAPP or raises the existing
%      singleton*.
%
%      H = PADAPP returns the handle to a new PADAPP or the handle to
%      the existing singleton*.
%
%      PADAPP('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in PADAPP.M with the given input arguments.
%
%      PADAPP('Property','Value',...) creates a new PADAPP or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before PADApp_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to PADApp_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help PADApp

% Last Modified by GUIDE v2.5 11-Feb-2018 19:28:01

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @PADApp_OpeningFcn, ...
                   'gui_OutputFcn',  @PADApp_OutputFcn, ...
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


% --- Executes just before PADApp is made visible.
function PADApp_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to PADApp (see VARARGIN)

% Choose default command line output for PADApp
handles.output = hObject;

FillCalibrationParameters(handles);

addpath('D:\TCC\Desenvolvimento\AnguloAtaque\matlab');

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes PADApp wait for user response (see UIRESUME)
% uiwait(handles.figure1);


% --- Outputs from this function are returned to the command line.
function varargout = PADApp_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on button press in btnCalibracaoAplicar.
function btnCalibracaoAplicar_Callback(hObject, eventdata, handles)
% hObject    handle to btnCalibracaoAplicar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
AplicarParametrosCalibracao(hObject, handles);


function txtCalibracaoA_Callback(hObject, eventdata, handles)
% hObject    handle to txtCalibracaoA (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txtCalibracaoA as text
%        str2double(get(hObject,'String')) returns contents of txtCalibracaoA as a double


% --- Executes during object creation, after setting all properties.
function txtCalibracaoA_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txtCalibracaoA (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end



function txtCalibracaoB_Callback(hObject, eventdata, handles)
% hObject    handle to txtCalibracaoB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txtCalibracaoB as text
%        str2double(get(hObject,'String')) returns contents of txtCalibracaoB as a double


% --- Executes during object creation, after setting all properties.
function txtCalibracaoB_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txtCalibracaoB (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btnCalibracaoResetar.
function btnCalibracaoResetar_Callback(hObject, eventdata, handles)
% hObject    handle to btnCalibracaoResetar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
FillCalibrationParameters(handles);


% --- Executes on button press in btnCalibracaoSalvar.
function btnCalibracaoSalvar_Callback(hObject, eventdata, handles)
% hObject    handle to btnCalibracaoSalvar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

a = str2num(get(handles.txtCalibracaoA,'String'));
b = str2num(get(handles.txtCalibracaoB,'String'));
c = str2num(get(handles.txtCalibracaoC,'String'));

assignin('base','a',a);
assignin('base','b',b);
assignin('base','c',c);

save('config.mat', 'a', 'b', 'c');

function txtComunicacaoPorta_Callback(hObject, eventdata, handles)
% hObject    handle to txtComunicacaoPorta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txtComunicacaoPorta as text
%        str2double(get(hObject,'String')) returns contents of txtComunicacaoPorta as a double


% --- Executes during object creation, after setting all properties.
function txtComunicacaoPorta_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txtComunicacaoPorta (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btnComunicacaoConectar.
function btnComunicacaoConectar_Callback(hObject, eventdata, handles)
% hObject    handle to btnComunicacaoConectar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if (~isfield(handles,'s'))
    [s, flag] = setup_serial(get(handles.txtComunicacaoPorta,'String'));
    if (flag == 1)
        pause(1);
        fprintf(s, 'i');
        pause(1);
        leitura = fscanf(s,'%d;', [4 1]);
        if (length(leitura)==4)
            
            % Cartão SD
            HabilitaCheckBox(handles.chbCartaoSD, leitura(1));
            HabilitaTexto(handles.txtNomeArquivo, leitura(1));
            % Orientação 
            HabilitaCheckBox(handles.chbOrientacao, leitura(2));
            % Aceleracao
            HabilitaCheckBox(handles.chbAceleracao, leitura(2));
            % Pressao
            HabilitaCheckBox(handles.chbPressaoAltitude, leitura(3));
            % Velocidade da Roda
            HabilitaCheckBox(handles.chbVelocidadeRoda, leitura(4));
            
            % Atualiza Botao
            handles.s = s;
            set(hObject, 'String', 'Desconectar');
            set(handles.btnTelemetriaIniciar, 'String', 'Iniciar');
            set(handles.btnTelemetriaIniciar, 'Enable', 'on');
            guidata(hObject, handles);
            
            LimparTelaTelemetria(handles);
            
        	drawnow;
        else
            fclose(s);
        end
    end
else
	fclose(handles.s);
    handles = rmfield(handles, 's');
    set(hObject, 'String', 'Conectar');
    guidata(hObject, handles);
    drawnow;
end

function HabilitaCheckBox(hObject, bHabilita)
    if (bHabilita == 1)
        set(hObject, 'Enable', 'on');
    else
        set(hObject, 'Value',   0);
        set(hObject, 'Enable',  'off');
    end

function HabilitaTexto(hObject, bHabilita)
    if (bHabilita == 1)
        set(hObject, 'Enable', 'on');
    else
        set(hObject, 'Enable',  'off');
    end

function txtCalibracaoC_Callback(hObject, eventdata, handles)
% hObject    handle to txtCalibracaoC (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txtCalibracaoC as text
%        str2double(get(hObject,'String')) returns contents of txtCalibracaoC as a double


% --- Executes during object creation, after setting all properties.
function txtCalibracaoC_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txtCalibracaoC (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in chbTelemetria.
function chbTelemetria_Callback(hObject, eventdata, handles)
% hObject    handle to chbTelemetria (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of chbTelemetria


% --- Executes on button press in chbCartaoSD.
function chbCartaoSD_Callback(hObject, eventdata, handles)
% hObject    handle to chbCartaoSD (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of chbCartaoSD



function txtNomeArquivo_Callback(hObject, eventdata, handles)
% hObject    handle to txtNomeArquivo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'String') returns contents of txtNomeArquivo as text
%        str2double(get(hObject,'String')) returns contents of txtNomeArquivo as a double


% --- Executes during object creation, after setting all properties.
function txtNomeArquivo_CreateFcn(hObject, eventdata, handles)
% hObject    handle to txtNomeArquivo (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: edit controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on button press in btnTelemetriaIniciar.
function btnTelemetriaIniciar_Callback(hObject, eventdata, handles)
% hObject    handle to btnTelemetriaIniciar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
tramitindo  = get(handles.chbTelemetria, 'Value');
gravando    = get(handles.chbCartaoSD, 'Value');
velocidadear    = get(handles.chbVelocidadeAr, 'Value');
orientacao    = get(handles.chbOrientacao, 'Value');
pressao    = get(handles.chbPressaoAltitude, 'Value');
correnteeletrica    = get(handles.chbCorrenteEletrica, 'Value');
velocidaderoda    = get(handles.chbVelocidadeRoda, 'Value');
aceleracao    = get(handles.chbAceleracao, 'Value');
nomearquivo = get(handles.txtNomeArquivo, 'String');
flushinput(handles.s)
fprintf(handles.s, strcat(num2str(tramitindo), ';', num2str(gravando), ';', num2str(velocidadear), ';', num2str(orientacao), ';', num2str(pressao), ';', num2str(correnteeletrica), ';', num2str(velocidaderoda), ';', num2str(aceleracao), ';', nomearquivo, '.txt|'));
leitura = fscanf(handles.s,'%c');
if (length(leitura) > 0) && (leitura(1) == 111)
    set(handles.btnTelemetriaIniciar,	'Enable', 'off');
    set(handles.btnTelemetriaParar,	'Enable', 'on');

    DefinirCorPainel(handles.pVelocidadeAr, velocidadear);
    DefinirCorPainel(handles.pOrientacao, orientacao);
    DefinirCorPainel(handles.pAltitude, pressao);
    DefinirCorPainel(handles.pCorrenteEletrica, correnteeletrica);
    DefinirCorPainel(handles.pVelocidadeRoda, velocidaderoda);
    DefinirCorPainel(handles.pAceleracao, aceleracao);

    drawnow;

    handles.tramitindo  = tramitindo;
    handles.gravando    = gravando;
    handles.velocidadear    = velocidadear;
    handles.orientacao    = orientacao;
    handles.pressao    = pressao;
    handles.correnteeletrica    = correnteeletrica;
    handles.velocidaderoda    = velocidaderoda;
    handles.aceleracao    = aceleracao;

    guidata(hObject, handles);

    if (handles.tramitindo == 1)
        ColetarDados(handles);
    end
       
else
    msgbox('Não foi possível iniciar a comunicação. Teste novamente!');
end

function DefinirCorPainel(hObject, Habilitado)
    if (Habilitado == 1)
        color = [0 1 0]; % verde
    else
        color = [1 0 0]; % vermelho
    end
    set(hObject, 'BackgroundColor', color);
% --- Executes on button press in chbVelocidadeAr.
function chbVelocidadeAr_Callback(hObject, eventdata, handles)
% hObject    handle to chbVelocidadeAr (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of chbVelocidadeAr


% --- Executes on button press in chbOrientacao.
function chbOrientacao_Callback(hObject, eventdata, handles)
% hObject    handle to chbOrientacao (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of chbOrientacao


% --- Executes on button press in chbPressaoAltitude.
function chbPressaoAltitude_Callback(hObject, eventdata, handles)
% hObject    handle to chbPressaoAltitude (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of chbPressaoAltitude


% --- Executes on button press in chbVelocidadeRoda.
function chbVelocidadeRoda_Callback(hObject, eventdata, handles)
% hObject    handle to chbVelocidadeRoda (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of chbVelocidadeRoda


% --- Executes on button press in chbAceleracao.
function chbAceleracao_Callback(hObject, eventdata, handles)
% hObject    handle to chbAceleracao (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of chbAceleracao


% --- Executes on selection change in listbox1.
function listbox1_Callback(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns listbox1 contents as cell array
%        contents{get(hObject,'Value')} returns selected item from listbox1


% --- Executes during object creation, after setting all properties.
function listbox1_CreateFcn(hObject, eventdata, handles)
% hObject    handle to listbox1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes when selected object is changed in uibuttongroup1.
function uibuttongroup1_SelectionChangedFcn(hObject, eventdata, handles)
% hObject    handle to the selected object in uibuttongroup1 
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


% --- Executes on button press in chbCorrenteEletrica.
function chbCorrenteEletrica_Callback(hObject, eventdata, handles)
% hObject    handle to chbCorrenteEletrica (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hint: get(hObject,'Value') returns toggle state of chbCorrenteEletrica

function FillCalibrationParameters(handles)
    load('config.mat');
    set(handles.txtCalibracaoA,'String', num2str(a));
    set(handles.txtCalibracaoB,'String', num2str(b));
    set(handles.txtCalibracaoC,'String', num2str(c));
    
function AplicarParametrosCalibracao(hObject, handles)

handles.a = str2num(get(handles.txtCalibracaoA,'String'));
handles.b = str2num(get(handles.txtCalibracaoB,'String'));
handles.c = str2num(get(handles.txtCalibracaoC,'String'));

set(handles.btnComunicacaoConectar, 'Enable', 'on');
drawnow;

% Update handles structure
guidata(hObject, handles);


% --- Executes on button press in btnTelemetriaParar.
function btnTelemetriaParar_Callback(hObject, eventdata, handles)
% hObject    handle to btnTelemetriaParar (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)fprintf(handles.s, strcat(tramitindo, ';', gravando, ';', velocidadear, ';', orientacao, ';', pressao, ';', correnteeletrica, ';', velocidaderoda, ';', aceleracao, ';', nomearquivo, ';', '.txt@'));
handles.tramitindo = 0;
guidata(hObject, handles);
pause(3);
fprintf(handles.s, 'p');
pause(1);
flushinput(handles.s);
fprintf(handles.s, 'p');
pause(1);
try
    leitura = fscanf(handles.s, '%c');
    if (length(leitura) > 0) && (leitura(1) == 111) 
        set(handles.btnTelemetriaIniciar,	'Enable', 'on');
        set(handles.btnTelemetriaParar,     'Enable', 'off');
        LimparTelaTelemetria(handles);
        drawnow;
    end
catch
    msgbox('ERRO');
end

function ColetarDados(handles)
    NUM_AMOSTRAS	= 100;
    array_tempomili	= zeros(NUM_AMOSTRAS, 1);
    array_velocidadear	= zeros(NUM_AMOSTRAS, 1);
    array_altitude      = zeros(NUM_AMOSTRAS, 1); 
    array_corrente      = zeros(NUM_AMOSTRAS, 1); 
    array_velocidaderoda	= zeros(NUM_AMOSTRAS, 1); 
    array_aceleracao_x      = zeros(NUM_AMOSTRAS, 1); 
    array_aceleracao_y      = zeros(NUM_AMOSTRAS, 1); 
    array_aceleracao_z      = zeros(NUM_AMOSTRAS, 1); 
    flushinput(handles.s);
    while (handles.tramitindo == 1)
        try
            leitura = fscanf(handles.s,'%d;', [12 1]);
            if (length(leitura(:,1))==12)
                % Carrega Variaveis Internas
                tempomili       = leitura(1);
                % Carrega Array
                array_tempomili     = [ array_tempomili(2:NUM_AMOSTRAS)     ; tempomili ];
                % Carrega Textos
                set(handles.txtTempo,               'String', num2str(tempomili));

                if (handles.velocidadear == 1)
                    % Carrega Variaveis Internas
                    pressaodiferencial  = double(leitura(2))/100;
                    % Calcula
                    velocidadear        = handles.a*(pressaodiferencial^2)+handles.b*pressaodiferencial+handles.c;
                    % Carrega Array
                    array_velocidadear	= [ array_velocidadear(2:NUM_AMOSTRAS)  ; velocidadear ];
                    % Carrega Textos
                    set(handles.txtVelocidadeAr,        'String', num2str(velocidadear));
                    % Carrega Axes
                    axes(handles.axVelocidadeAr);
                    plot(1:NUM_AMOSTRAS, array_velocidadear(1:NUM_AMOSTRAS));
                end

                if (handles.orientacao == 1)
                    % Carrega Variaveis Internas
                    orientacao_q_w	= double(leitura(3))/100;
                    orientacao_q_x	= double(leitura(4))/100;
                    orientacao_q_y	= double(leitura(5))/100;
                    orientacao_q_z	= double(leitura(6))/100;
                    % Calcula
                    orientacao_euler    = quat2eul([orientacao_q_w orientacao_q_x orientacao_q_y orientacao_q_z])*180/pi;
                    % Carrega Axes
                    axes(handles.axOrientacao);
                    c130(0,0,0,'yaw', 0, 'pitch', orientacao_euler(2)-180, 'roll', orientacao_euler(3)-180);
                end

                if (handles.pressao == 1)
                    % Carrega Variaveis Internas
                    pressao         = leitura(7);
                    % Calcula
                    altitude            = 44330*(1-(double(pressao)/101325)^(1/5.255));
                    % Carrega Array
                    array_altitude      = [ array_altitude(2:NUM_AMOSTRAS)      ; altitude ];
                    % Carrega Textos
                    set(handles.txtAltitudeLocal,	'String', num2str(altitude));
                    % Carrega Axes
                    axes(handles.axAltitudeLocal);
                    plot(1:NUM_AMOSTRAS,array_altitude(1:NUM_AMOSTRAS));
                end

                if (handles.correnteeletrica == 1)
                    % Carrega Variaveis Internas
                    corrente_eletrica	= leitura(8);
                    % Carrega Array
                    array_corrente      = [ array_corrente(2:NUM_AMOSTRAS)      ; corrente_eletrica ];
                    % Carrega Textos
                    set(handles.txtCorrenteEletrica,	'String', num2str(corrente_eletrica));
                    % Carrega Axes
                    axes(handles.axCorrenteEletrica);
                    plot(1:NUM_AMOSTRAS, array_corrente(1:NUM_AMOSTRAS));
                end

                if (handles.velocidaderoda == 1)
                    % Carrega Variaveis Internas
                    velocidaderoda	= leitura(9);
                    % Carrega Array
                    array_velocidaderoda	= [ array_velocidaderoda(2:NUM_AMOSTRAS)      ; velocidaderoda ];
                    % Carrega Textos
                    set(handles.txtVelocidadeRoda,	'String', num2str(velocidaderoda));
                    % Carrega Axes
                    axes(handles.axVelocidadeRoda);
                    plot(1:NUM_AMOSTRAS, array_velocidaderoda(1:NUM_AMOSTRAS));
                end

                if (handles.aceleracao == 1)
                    % Carrega Variaveis Internas
                    aceleracao_x	= leitura(10);
                    aceleracao_y	= leitura(11);
                    aceleracao_z	= leitura(12);
                    % Carrega Array
                    array_aceleracao_x	= [ array_aceleracao_x(2:NUM_AMOSTRAS)      ; aceleracao_x ];
                    array_aceleracao_y	= [ array_aceleracao_y(2:NUM_AMOSTRAS)      ; aceleracao_y ];
                    array_aceleracao_z	= [ array_aceleracao_z(2:NUM_AMOSTRAS)      ; aceleracao_z ];
                    % Carrega Axes
                    axes(handles.axAceleracao);
                    plot(1:NUM_AMOSTRAS, array_aceleracao_x(1:NUM_AMOSTRAS), 1:NUM_AMOSTRAS,array_aceleracao_y(1:NUM_AMOSTRAS),1:NUM_AMOSTRAS,array_aceleracao_z(1:NUM_AMOSTRAS));
                end
                drawnow;
                handles = guidata(gcbo);
                flushinput(handles.s);
            end
        catch
            
        end
    end
    
function LimparTelaTelemetria(handles)

    LimparTexto(handles.txtVelocidadeAr);
    LimparTexto(handles.txtAltitudeLocal);
    LimparTexto(handles.txtCorrenteEletrica);
    LimparTexto(handles.txtVelocidadeRoda);

    LimparAxes(handles.axVelocidadeAr);
    LimparAxes(handles.axOrientacao);
    LimparAxes(handles.axAltitudeLocal);
    LimparAxes(handles.axCorrenteEletrica);
    LimparAxes(handles.axVelocidadeRoda);
    LimparAxes(handles.axAceleracao);

    LimparPanel(handles.pVelocidadeAr);
    LimparPanel(handles.pOrientacao);
    LimparPanel(handles.pAltitude);
    LimparPanel(handles.pCorrenteEletrica);
    LimparPanel(handles.pVelocidadeRoda);
    LimparPanel(handles.pAceleracao);

function LimparTexto(hObject)
    set(hObject, 'String', 'XXXX');
function LimparAxes(hObject)
    cla(hObject);
function LimparPanel(hObject)
    set(hObject, 'BackgroundColor', [1 1 1]);
