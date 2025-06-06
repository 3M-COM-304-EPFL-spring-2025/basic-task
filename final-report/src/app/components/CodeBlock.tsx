// components/CodeBlock.tsx
import { useState } from "react";
import { ChevronDown, ChevronRight, ExternalLink } from "lucide-react";

interface CodeBlockProps {
  code: string;
  title?: string;
  githubUrl?: string;
}

const CodeBlock: React.FC<CodeBlockProps> = ({
  code,
  title = "Code Snippet",
  githubUrl,
}) => {
  const [isVisible, setIsVisible] = useState(false);

  return (
    <div className="my-6 border rounded-lg overflow-hidden hover:cursor-pointer">
      <div
        className="flex items-center justify-between px-4 py-2 bg-gray-900 text-gray-100 text-sm font-mono"
        onClick={() => setIsVisible(!isVisible)}
      >
        <div className="flex items-center gap-1 text-left">
          {isVisible ? (
            <ChevronDown className="w-4 h-4" />
          ) : (
            <ChevronRight className="w-4 h-4" />
          )}
          {title}
        </div>
        {githubUrl && (
          <a
            href={githubUrl}
            target="_blank"
            rel="noopener noreferrer"
            className="text-blue-400 hover:underline flex items-center gap-1"
          >
            <ExternalLink className="w-4 h-4" />
            View on GitHub
          </a>
        )}
      </div>
      {isVisible && (
        <pre className="p-4 bg-gray-800 text-white overflow-x-auto">
          {code.trim()}
        </pre>
      )}
    </div>
  );
};

export default CodeBlock;
