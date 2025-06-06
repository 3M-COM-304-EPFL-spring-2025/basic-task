import React from 'react';

interface SectionHeaderProps {
  title: string;
}

const SectionHeader: React.FC<SectionHeaderProps & { id: string }> = ({ title, id }) => {
  return (
    <h2 id={id} className="text-3xl font-bold text-gray-900 mt-8 mb-4">
      {title}
    </h2>
  );
};

export default SectionHeader;
