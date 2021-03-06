%CLASS_INTERFACE Example MATLAB class wrapper to an underlying C++ class
classdef foveated_stereo < handle
    properties (SetAccess = private, Hidden = true)
        objectHandle; % Handle to the underlying C++ class instance
    end
    methods
        %% Constructor - Create a new C++ class instance 
        function this = foveated_stereo(varargin)
            this.objectHandle = foveated_stereo_interface_mex('new', varargin{:});
        end
        
        %% Destructor - Destroy the C++ class instance
        function delete(this)
            foveated_stereo_interface_mex('delete', this.objectHandle);
        end

        %% To cortical
        function varargout = to_cortical(this, varargin)
            [varargout{1:nargout}] = foveated_stereo_interface_mex('to_cortical', this.objectHandle, varargin{:});
        end
        
        %% To cartesian
        function varargout = to_cartesian(this, varargin)
            [varargout{1:nargout}] = foveated_stereo_interface_mex('to_cartesian', this.objectHandle, varargin{:});
        end
        
        %% Rectify stereo
        function varargout = rectify_stereo(this, varargin)
            [varargout{1:nargout}] = foveated_stereo_interface_mex('rectify_stereo', this.objectHandle, varargin{:});
        end
        %% Get uncertainty
        function varargout = get_uncertainty(this, varargin)
            [varargout{1:nargout}] = foveated_stereo_interface_mex('get_uncertainty', this.objectHandle, varargin{:});
        end
        
        
        %% LogPolar disparity maps 
        function varargout = get_logpolar_warp_maps(this, varargin)
            [varargout{1:nargout}] = foveated_stereo_interface_mex('get_logpolar_warp_maps', this.objectHandle, varargin{:});
        end
        
        %% LogPolar delta disparity maps
        function varargout = get_delta_logpolar_warp_maps(this, varargin)
            [varargout{1:nargout}] = foveated_stereo_interface_mex('get_delta_logpolar_warp_maps', this.objectHandle, varargin{:});
        end
        
         %% warpLogPolar delta disparity maps
        function varargout = warp_logpolar(this, varargin)
            [varargout{1:nargout}] = foveated_stereo_interface_mex('warp_logpolar', this.objectHandle, varargin{:});
        end
        
        %% warpCartesianLogPolar delta disparity maps
        function varargout = warp_cartesian_logpolar(this, varargin)
            [varargout{1:nargout}] = foveated_stereo_interface_mex('warp_cartesian_logpolar', this.objectHandle, varargin{:});
        end
        
        %% computeDisparityMaps disparity maps
        function varargout = compute_disparity_map(this, varargin)
            [varargout{1:nargout}] = foveated_stereo_interface_mex('compute_disparity_map', this.objectHandle, varargin{:});
        end      
        
        %% filter disparity maps
        function varargout = filter(this, varargin)
            [varargout{1:nargout}] = foveated_stereo_interface_mex('filter', this.objectHandle, varargin{:});
        end 
        
        %% get egosphere
        function varargout = get_egosphere(this, varargin)
            [varargout{1:nargout}] = foveated_stereo_interface_mex('get_egosphere', this.objectHandle, varargin{:});
        end
        
    end
end