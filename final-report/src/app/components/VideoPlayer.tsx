"use client";
import React from "react";
import ReactPlayer from "react-player";

interface VideoPlayerProps {
  src: string; // local video file path or URL
  width?: string | number;
  height?: string | number;
}

const VideoPlayer: React.FC<VideoPlayerProps> = ({
  src,
}) => {
  return (
    <div className="player-wrapper" style={{ width: "100%", height: "100%" }}>
      <ReactPlayer
        url={src}
        controls
        width="100%"
        playing={false}
      />
    </div>
  );
};

export default VideoPlayer;
