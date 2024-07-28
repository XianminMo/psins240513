function demop(n, n1)
% Demo-PSINS list.
%
% Prototype: demop(n, n1)
% Inputs: n, n1 - n or filenames wildcards
%
% Examples
%    demop;          % display all demo files
%    demop(10);      % open 10th demo file
%    demop('ekf');   % display demo files with wildcards '*ekf*.m'
%    demop('ekf',3); % open 3rd 'ekf'-demo file

% Copyright(c) 2009-2024, by Gongmin Yan, All rights reserved.
% Northwestern Polytechnical University, Xi An, P.R.China
% 16/01/2024
global glv
    if nargin<2, n1=0; end
    if nargin<1, n=0; end
    if ischar(n), name=['\demos\*',n,'*.m']; n=n1; else, name='\demos\*.m'; end
    [fnames,m] = dirfile([glv.rootpath,name]);
    if n==0
        for k=1:m
            fnames{k}=sprintf('%3d  %s',k,fnames{k});
        end
        disp(fnames);
    else
        open(fnames{n});
    end
%     fid = fopen([glv.rootpath,'\demos\psinsdemo.m'],'w');
%     for k=1:m
%         fprintf(fid, '%s\r\n', fnames{k});
%     end
%     fclose(fid);