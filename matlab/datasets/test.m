function [] = test()
%TEST Demonstrate how to use parsePfm.  Note that if "tonemap" does not
%work, then you do not have the "Image Processing Toolbox" installed.  This
%does not affect the usage of parsePfm.

    TITLE_SZ = 18;

    fprintf('Loading 3 images...\n');
    lenaGray      = parsePfm( 'test_images/lena_gray.pfm' );
    lenaColor     = parsePfm( 'test_images/lena_color.pfm');
    hdrProbeColor = parsePfm( 'test_images/hdr_probe.pfm' );
    fprintf('Done loading image.\n');
    
    fprintf('Making plots...\n');
    figure;
    imagesc(lenaGray); axis image; colormap gray;
    t = title(sprintf('Grayscale Lena Image: %u width, %u height',...
        size(lenaGray,1), size(lenaGray,2)) );
    set(t, 'FontSize', TITLE_SZ);
    
    figure;
    imagesc(lenaColor); axis image;
    t = title(sprintf('Color Lena Image: %u width, %u height',...
        size(lenaColor,1), size(lenaColor,2)) );
    set(t, 'FontSize', TITLE_SZ);
    
    figure;
    imagesc(tonemap(hdrProbeColor)); axis image;
    t = title(sprintf('Color HDR Image: %u width, %u height',...
        size(hdrProbeColor,1), size(hdrProbeColor,2)) );    
    set(t, 'FontSize', TITLE_SZ);
    fprintf('Done making plots.\n');

end

