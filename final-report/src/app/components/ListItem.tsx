import React from 'react';

interface ListItemProps {
  children: React.ReactNode;
}

const ListItem: React.FC<ListItemProps & { href?: string }> = ({ children, href }) => {
  return href ? (
    <li className="text-gray-700 list-inside list-disc mb-2">
      <a href={href} className="text-blue-500 hover:underline">
        {children}
      </a>
    </li>
  ) : (
    <li className="text-gray-700 list-inside list-disc mb-2">{children}</li>
  );
};

export default ListItem;
