// components/Figure.tsx
import Image, { StaticImageData } from "next/image";

type FigureProps = {
    src: StaticImageData;
    alt: string;
    caption: string;
    width: number;
    height: number;
};

const Figure = ({ src, alt, caption, width, height }: FigureProps) => {
  return (
    <figure className="text-center py-4">
      <Image
        src={src}
        alt={alt}
        width={width}
        height={height}
        className="mx-auto rounded-lg"
      />
      <figcaption className="mt-2 text-sm text-gray-600">{caption}</figcaption>
    </figure>
  );
};

export default Figure;
