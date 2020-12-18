function makeMovie(f, fps, filename)
    v = VideoWriter(filename, 'MPEG-4');
    v.FrameRate = fps;
    open(v);
    writeVideo(v, f);
    close(v);
end