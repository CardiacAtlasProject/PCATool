function varargout = SingleFrameModeViewer(varargin)
% SINGLEFRAMEMODEVIEWER MATLAB code for SingleFrameModeViewer.fig
%      SINGLEFRAMEMODEVIEWER, by itself, creates a new SINGLEFRAMEMODEVIEWER or raises the existing
%      singleton*.
%
%      H = SINGLEFRAMEMODEVIEWER returns the handle to a new SINGLEFRAMEMODEVIEWER or the handle to
%      the existing singleton*.
%
%      SINGLEFRAMEMODEVIEWER('CALLBACK',hObject,eventData,handles,...) calls the local
%      function named CALLBACK in SINGLEFRAMEMODEVIEWER.M with the given input arguments.
%
%      SINGLEFRAMEMODEVIEWER('Property','Value',...) creates a new SINGLEFRAMEMODEVIEWER or raises the
%      existing singleton*.  Starting from the left, property value pairs are
%      applied to the GUI before SingleFrameModeViewer_OpeningFcn gets called.  An
%      unrecognized property name or invalid value makes property application
%      stop.  All inputs are passed to SingleFrameModeViewer_OpeningFcn via varargin.
%
%      *See GUI Options on GUIDE's Tools menu.  Choose "GUI allows only one
%      instance to run (singleton)".
%
% See also: GUIDE, GUIDATA, GUIHANDLES

% Edit the above text to modify the response to help SingleFrameModeViewer

% Last Modified by GUIDE v2.5 24-Jan-2018 14:55:43

% Begin initialization code - DO NOT EDIT
gui_Singleton = 1;
gui_State = struct('gui_Name',       mfilename, ...
                   'gui_Singleton',  gui_Singleton, ...
                   'gui_OpeningFcn', @SingleFrameModeViewer_OpeningFcn, ...
                   'gui_OutputFcn',  @SingleFrameModeViewer_OutputFcn, ...
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


% --- Executes just before SingleFrameModeViewer is made visible.
function SingleFrameModeViewer_OpeningFcn(hObject, eventdata, handles, varargin)
% This function has no output args, see OutputFcn.
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)
% varargin   command line arguments to SingleFrameModeViewer (see VARARGIN)

if( length(varargin) ~= 2 )
    error('Inputs: shape_file.mat, ''shape''|''diff''.');
end

% Choose default command line output for SingleFrameModeViewer
handles.output = hObject;
handles.model = importdata(varargin{1});
handles.surf_face = importdata('SurfaceFaces.mat');

% set axis
handles.ax.Color = 'none';

% populate listbox
pct = handles.model.VAR_EXPL;
handles.mode_list.String = cellfun(@(x) sprintf('Mode %d (%.2f%%)', ...
    x(1), x(2)), num2cell([(1:numel(pct))' pct],2), 'UniformOutput', false);
handles.mode_list.Value = 1;

% set title
handles.axis_panel.Title = sprintf('Mode %d: %.2f SD from MEAN SHAPE', ...
    1, 0);

% plot the mean surface

if( strcmpi(varargin{2},'shape') )
    handles.get_mode = @(mode, d) ...
        handles.model.MEAN + d * handles.model.COEFF(:,mode)';
    handles.change_fcn = @replace_Surface;
    handles = create_Surface(handles);
elseif( strcmpi(varargin{2},'diff') )
    handles.get_mode = @(mode, d) ...
        handles.model.MEAN + d * handles.model.COEFF(:,mode)';
    handles.change_fcn = @replace_DiffSurface;
    handles = create_DiffSurface(handles);
    
    handles.menuFileExportVTK.Enable = 'off';
elseif( strcmpi(varargin{2},'diff_as_shape') )
    
    handles.get_mode = @(mode, d) ...
        (handles.model.MEAN + d * handles.model.COEFF(:,mode)') + ...
        handles.model.ED_SHAPE;
    
    handles.change_fcn = @replace_Surface;
    handles = create_Surface(handles);
elseif( strcmpi(varargin{2},'diff_as_deformable_model') )
        
    handles.get_mode = @(mode, d) ...
        (handles.model.MEAN + d * handles.model.COEFF(:,mode)') + ...
        handles.model.ED_SHAPE;
    
    %handles.change_fcn = @replace_DeformedSurface;
    handles.change_fcn = @replace_Surface;
    handles = create_DeformedSurface(handles);
    
    handles.menuFileExportVTK.Enable = 'off';
    
elseif( strcmpi(varargin{2},'edes_single_frame') )
    
    handles.get_mode = @(mode, d) ...
        handles.model.MEAN + d * handles.model.COEFF(:,mode)';
    handles.change_fcn = @replace_EDESSurface;
    handles = create_EDESSurface(handles);
    
    handles.menuFileExportVTK.Enable = 'off';
else
    error('Unknown type');
end

% Update handles structure
guidata(hObject, handles);

% UIWAIT makes SingleFrameModeViewer wait for user response (see UIRESUME)
% uiwait(handles.figure1);

cameratoolbar;

% --- Outputs from this function are returned to the command line.
function varargout = SingleFrameModeViewer_OutputFcn(hObject, eventdata, handles) 
% varargout  cell array for returning output args (see VARARGOUT);
% hObject    handle to figure
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Get default command line output from handles structure
varargout{1} = handles.output;


% --- Executes on selection change in mode_list.
function mode_list_Callback(hObject, eventdata, handles)
% hObject    handle to mode_list (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: contents = cellstr(get(hObject,'String')) returns mode_list contents as cell array
%        contents{get(hObject,'Value')} returns selected item from mode_list

% set the title
handles.axis_panel.Title = sprintf('Mode %d: %.2f SD from MEAN SHAPE', ...
    handles.mode_list.Value, get(hObject,'Value'));

% change
handles.change_fcn(handles);



% --- Executes during object creation, after setting all properties.
function mode_list_CreateFcn(hObject, eventdata, handles)
% hObject    handle to mode_list (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: listbox controls usually have a white background on Windows.
%       See ISPC and COMPUTER.
if ispc && isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor','white');
end


% --- Executes on slider movement.
function var_slide_Callback(hObject, eventdata, handles)
% hObject    handle to var_slide (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

% Hints: get(hObject,'Value') returns position of slider
%        get(hObject,'Min') and get(hObject,'Max') to determine range of slider

% set the title
handles.axis_panel.Title = sprintf('Mode %d: %.2f SD from MEAN SHAPE', ...
    handles.mode_list.Value, get(hObject,'Value'));

% change
handles.change_fcn(handles);


% -- PLOT ANNOTATION
function plot_Annotation(handles)
 
% anatomical labeling
x = handles.ax.XLim(2);
zs = handles.ax.ZLim;
ys = handles.ax.YLim;
 
%AAY 27 Mar 2020 seems the axes were labelled incorrectly. Relabel these so y axis points to septum
% text(x, 0, zs(2), 'S', 'Parent', handles.ax, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
% text(x, ys(2), 0, 'I', 'Parent', handles.ax, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
% text(x, 0, zs(1), 'L', 'Parent', handles.ax, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
% text(x, ys(1), 0, 'A', 'Parent', handles.ax, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
text(x, 0, zs(2), 'I', 'Parent', handles.ax, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
text(x, ys(2), 0, 'S', 'Parent', handles.ax, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
text(x, 0, zs(1), 'A', 'Parent', handles.ax, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
text(x, ys(1), 0, 'L', 'Parent', handles.ax, 'FontWeight', 'bold', 'HorizontalAlignment', 'center');
 
plot3([x x], ys + [10 -10], [0 0], ':k', 'Parent', handles.ax);
plot3([x x], [0 0], zs + [10 -10], ':k', 'Parent', handles.ax);

% -- CREATE SURFACE
function handles = create_Surface(handles)

mode = handles.mode_list.Value;
d = handles.var_slide.Value * sqrt(handles.model.LATENT(handles.mode_list.Value));
P = reshape(handles.get_mode(mode, d), 3, [])';

handles.patch_endo = patch( 'Faces', handles.surf_face.Endo, ...
    'Vertices', P, 'EdgeColor', 'none', ...
    'FaceAlpha', 0.3, 'FaceColor', 'r', 'Parent', handles.ax );
hold(handles.ax,'on');
handles.patch_epi = patch( 'Faces', handles.surf_face.Epi, ...
    'Vertices', P, 'EdgeColor', 'none', ...
    'FaceAlpha', 0.3, 'FaceColor', 'b', 'Parent', handles.ax );

axis(handles.ax, 'equal');
axis(handles.ax, 'vis3d');

plot_Annotation(handles);


% --- REPLACE DEFORMED SURFACE
function replace_DeformedSurface(handles)

mode = handles.mode_list.Value;
d = handles.var_slide.Value * sqrt(handles.model.LATENT(handles.mode_list.Value));
deformed_shape = reshape(handles.get_mode(mode, d), 3, [])';

P = reshape(handles.model.ED_SHAPE,3,[])';
handles.patch_endo.Vertices = P;
handles.patch_epi.Vertices = P;

% replace wireframe
handles = replace_Wirfeframe(deformed_shape, handles);

guidata(handles.figure1,handles);



% -- CREATE DEFORMABLE SURFACE
function handles = create_DeformedSurface(handles)

mode = handles.mode_list.Value;
d = handles.var_slide.Value * sqrt(handles.model.LATENT(handles.mode_list.Value));
P = reshape(handles.get_mode(mode, d), 3, [])';

handles.patch_endo = patch( 'Faces', handles.surf_face.Endo, ...
    'Vertices', P, 'EdgeColor', 'none', ...
    'FaceAlpha', 0.3, 'FaceColor', 'r', 'Parent', handles.ax );
hold(handles.ax,'on');
handles.patch_epi = patch( 'Faces', handles.surf_face.Epi, ...
    'Vertices', P, 'EdgeColor', 'none', ...
    'FaceAlpha', 0.3, 'FaceColor', 'b', 'Parent', handles.ax );

% the deformed wireframe
handles = get_Wireframe(reshape(handles.model.ED_SHAPE, 3,[])', handles);

axis(handles.ax, 'equal');
axis(handles.ax, 'vis3d');

plot_Annotation(handles);


% -- CREATE EDES SURFACE
function handles = create_EDESSurface(handles)

mode = handles.mode_list.Value;
d = handles.var_slide.Value * sqrt(handles.model.LATENT(handles.mode_list.Value));
S = handles.get_mode(mode, d);

N = length(S)/2;

P = reshape(S(1:N),3,[])';
handles.patch_endo = patch( 'Faces', handles.surf_face.Endo, ...
    'Vertices', P, 'EdgeColor', 'none', ...
    'FaceAlpha', 0.3, 'FaceColor', 'r', 'Parent', handles.ax );
hold(handles.ax,'on');
handles.patch_epi = patch( 'Faces', handles.surf_face.Epi, ...
    'Vertices', P, 'EdgeColor', 'none', ...
    'FaceAlpha', 0.3, 'FaceColor', 'b', 'Parent', handles.ax );

% the deformed wireframe
handles = get_Wireframe(reshape(S((N+1):end),3,[])', handles);

axis(handles.ax, 'equal');
axis(handles.ax, 'vis3d');

plot_Annotation(handles);

% --- REPLACE EDSSURFACE
function replace_EDESSurface(handles)

mode = handles.mode_list.Value;
d = handles.var_slide.Value * sqrt(handles.model.LATENT(handles.mode_list.Value));
S = handles.get_mode(mode, d);
N = length(S)/2;

P = reshape(S(1:N),3,[])';
handles.patch_endo.Vertices = P;
handles.patch_epi.Vertices = P;

% replace wireframe
handles = replace_Wirfeframe(reshape(S((N+1):end),3,[])', handles);

guidata(handles.figure1,handles);


% -- GET WIREFRAME
function handles = get_Wireframe(S, handles)

PointMap = importdata('PointMap.mat');
cidx = 1:3:29;
N = size(S,1)/2;

handles.circLines = [];
handles.longLines = [];
handles.connLines = [];
for i=1:numel(cidx)
    handles.circLines = [handles.circLines PointMap(cidx(i),:)'];
    handles.longLines = [handles.longLines PointMap(:,cidx(i))];
    if( i<numel(cidx) )
        handles.connLines = [handles.connLines; reshape(PointMap(cidx(i),cidx),[],1)];
    end
end
handles.connLines = [handles.connLines N+handles.connLines]';

P = S(handles.circLines,:);
handles.w_circ_endo = line(reshape(P(:,1),size(handles.circLines)), ...
    reshape(P(:,2),size(handles.circLines)), reshape(P(:,3),size(handles.circLines)), 'Color','k');

P = S(N+handles.circLines,:);
handles.w_circ_epi = line(reshape(P(:,1),size(handles.circLines)), ...
    reshape(P(:,2),size(handles.circLines)), reshape(P(:,3),size(handles.circLines)), 'Color','k');

P = S(handles.longLines,:);
handles.w_long_endo = line(reshape(P(:,1),size(handles.longLines)), ...
    reshape(P(:,2),size(handles.longLines)), reshape(P(:,3),size(handles.longLines)), 'Color','k');

P = S(N+handles.longLines,:);
handles.w_long_epi = line(reshape(P(:,1),size(handles.longLines)), ...
    reshape(P(:,2),size(handles.longLines)), reshape(P(:,3),size(handles.longLines)), 'Color','k');

P = S(handles.connLines,:);
handles.w_conn = line(reshape(P(:,1),size(handles.connLines)), ...
    reshape(P(:,2),size(handles.connLines)), reshape(P(:,3),size(handles.connLines)), 'Color','k');


% --- REPLACE WIREFRAME
function handles = replace_Wirfeframe(S, handles)

N = size(S,1)/2;

P = S(handles.circLines,:);
delete(handles.w_circ_endo);
handles.w_circ_endo = line(reshape(P(:,1),size(handles.circLines)), ...
    reshape(P(:,2),size(handles.circLines)), reshape(P(:,3),size(handles.circLines)), 'Color','k');

P = S(N+handles.circLines,:);
delete(handles.w_circ_epi);
handles.w_circ_epi = line(reshape(P(:,1),size(handles.circLines)), ...
    reshape(P(:,2),size(handles.circLines)), reshape(P(:,3),size(handles.circLines)), 'Color','k');

P = S(handles.longLines,:);
delete(handles.w_long_endo);
handles.w_long_endo = line(reshape(P(:,1),size(handles.longLines)), ...
    reshape(P(:,2),size(handles.longLines)), reshape(P(:,3),size(handles.longLines)), 'Color','k');

P = S(N+handles.longLines,:);
delete(handles.w_long_epi)
handles.w_long_epi = line(reshape(P(:,1),size(handles.longLines)), ...
    reshape(P(:,2),size(handles.longLines)), reshape(P(:,3),size(handles.longLines)), 'Color','k');

P = S(handles.connLines,:);
delete(handles.w_conn);
handles.w_conn = line(reshape(P(:,1),size(handles.connLines)), ...
    reshape(P(:,2),size(handles.connLines)), reshape(P(:,3),size(handles.connLines)), 'Color','k');




% --- REPLACE SURFACE
function replace_Surface(handles)

mode = handles.mode_list.Value;
d = handles.var_slide.Value * sqrt(handles.model.LATENT(handles.mode_list.Value));
P = reshape(handles.get_mode(mode, d), 3, [])';

handles.patch_endo.Vertices = P;
handles.patch_epi.Vertices = P;


% -- CREATE SURFACE
function handles = create_DiffSurface(handles)

% surface
P = reshape(handles.model.ED_SHAPE, 3, [])';
handles.patch_endo = patch( 'Faces', handles.surf_face.Endo, ...
    'Vertices', P, 'EdgeColor', 'none', ...
    'FaceAlpha', 0.3, 'FaceColor', 'r', 'Parent', handles.ax );
hold(handles.ax,'on');
handles.patch_epi = patch( 'Faces', handles.surf_face.Epi, ...
    'Vertices', P, 'EdgeColor', 'none', ...
    'FaceAlpha', 0.3, 'FaceColor', 'b', 'Parent', handles.ax );

axis(handles.ax, 'equal');
axis(handles.ax, 'vis3d');

% difference vector
mode = handles.mode_list.Value;
d = handles.var_slide.Value * sqrt(handles.model.LATENT(handles.mode_list.Value));
V = reshape(handles.get_mode(mode, d), 3, [])';

npts = size(V,1)/2;
handles.v_endo = quiver3(P(1:npts,1),P(1:npts,2),P(1:npts,3), ...
    V(1:npts,1),V(1:npts,2),V(1:npts,3), '-r');
handles.v_epi = quiver3(P((npts+1):end,1),P((npts+1):end,2),P((npts+1):end,3), ...
    V((npts+1):end,1),V((npts+1):end,2),V((npts+1):end,3), '-b');


plot_Annotation(handles);



% --- REPLACE DIFF SURFACE
function replace_DiffSurface(handles)

mode = handles.mode_list.Value;
d = handles.var_slide.Value * sqrt(handles.model.LATENT(handles.mode_list.Value));
V = reshape(handles.get_mode(mode, d), 3, [])';

npts = size(V,1)/2;

handles.v_endo.UData = V(1:npts,1);
handles.v_endo.VData = V(1:npts,2);
handles.v_endo.WData = V(1:npts,3);

handles.v_epi.UData = V((npts+1):end,1);
handles.v_epi.VData = V((npts+1):end,2);
handles.v_epi.WData = V((npts+1):end,3);



% --- Executes during object creation, after setting all properties.
function var_slide_CreateFcn(hObject, eventdata, handles)
% hObject    handle to var_slide (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    empty - handles not created until after all CreateFcns called

% Hint: slider controls usually have a light gray background.
if isequal(get(hObject,'BackgroundColor'), get(0,'defaultUicontrolBackgroundColor'))
    set(hObject,'BackgroundColor',[.9 .9 .9]);
end


% --- Executes when figure1 is resized.
function figure1_SizeChangedFcn(hObject, eventdata, handles)
% hObject    handle to figure1 (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if( isempty(handles) ), return; end

% fixed size listbox
% length = 20 characters
handles.figure1.Units = 'characters';
figpos = handles.figure1.Position;

handles.mode_list.Units = 'characters';
handles.mode_list.Position = [1 1 20 figpos(4)-2];

handles.axis_panel.Units = 'characters';
handles.axis_panel.Position = [22 1 figpos(3)-23 figpos(4)-2];

% panel
panpos = handles.axis_panel.Position;
varpos = handles.var_slide.Position;
handles.var_slide.Position = [varpos(1:2) panpos(3)-2*(varpos(1)) varpos(4)];

y = varpos(2)+varpos(4)+1;
h = panpos(4)-y-1;
w = panpos(3)-2;
handles.ax.Units = 'characters';
handles.ax.OuterPosition = [1 y w h];


% --------------------------------------------------------------------
function menuFileExportVTK_Callback(hObject, eventdata, handles)
% hObject    handle to menuFileExportVTK (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)


if( isempty(handles) ), return; end

% get the file
[fname,fpath] = uiputfile('*.vtk', 'Select a VTK file');
if( ~ischar(fname) ), return; end

% get mode
mode = handles.mode_list.Value;
d = handles.var_slide.Value * sqrt(handles.model.LATENT(handles.mode_list.Value));

% export
ExportShapeToVTK(handles.get_mode(mode, d), fullfile(fpath,fname), 'surf', handles.surf_face);

fprintf(1, 'Saved to %s\n', fullfile(fpath,fname));


% --------------------------------------------------------------------
function menuCreatePNGFiles_Callback(hObject, eventdata, handles)
% hObject    handle to menuCreatePNGFiles (see GCBO)
% eventdata  reserved - to be defined in a future version of MATLAB
% handles    structure with handles and user data (see GUIDATA)

if( isempty(handles) ), return; end

% get the directory
folder_name = uigetdir;
if( isempty(folder_name) || ~exist(folder_name,'dir') )
    return;
end

% we need to use other figure
newFig = figure('Color','w');
newAx = axes('OuterPosition',[0 0 1 1]);
HENDO = copyobj(handles.patch_endo, newAx);
HEPI = copyobj(handles.patch_epi, newAx);
axis(newAx, 'equal');
set(newAx,'Visible','off');
camlight('left');

% copyprop
props = {'CameraPosition', 'CameraTarget', 'CameraUpVector', 'CameraViewAngle'};
for pi=1:numel(props)
    set( newAx, props{pi}, get(handles.ax,props{pi}) );
end

fprintf(2,'Adjust the position & click any key to start outputing\n');
pause;


% ranges
ds = linspace(0, handles.var_slide.Min, 10);
y = linspace(0,handles.var_slide.Max,10); y = y(2:end);
ds = [ds fliplr(ds) y fliplr(y)];
for i=1:numel(ds)
    
    % change the surface
    handles.var_slide.Value = ds(i);
    replace_Surface(handles);
    
    % adjust the newFig
    HENDO.Vertices = handles.patch_endo.Vertices;
    HEPI.Vertices = handles.patch_epi.Vertices;
    drawnow;
    
    % output
    print( newFig, fullfile(folder_name, sprintf('mode%02d_%04d.png', handles.mode_list.Value, i)), ...
        '-dpng', '-r0', '-noui');
        
end

close(newFig);
